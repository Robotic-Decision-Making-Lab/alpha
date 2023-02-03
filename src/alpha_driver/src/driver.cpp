// Copyright 2023, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "alpha_driver/driver.hpp"

#include <vector>

#include "alpha_driver/device_id.hpp"
#include "alpha_driver/packet_id.hpp"

using namespace std::chrono_literals;

namespace alpha_driver
{

Driver::Driver()
: Node("AlphaDriver"),
  client_(this->get_logger())
{
  auto address_desc = rcl_interfaces::msg::ParameterDescriptor{};
  address_desc.description = "Full path to the serial port file";

  auto state_freq_desc = rcl_interfaces::msg::ParameterDescriptor{};
  state_freq_desc.description = "Frequency that the robot state should be published at";

  this->declare_parameter("address", "", address_desc);
  this->declare_parameter("freq", 1, state_freq_desc);

  const std::string address =
    this->get_parameter("address").get_parameter_value().get<std::string>();
  const int state_freq = this->get_parameter("freq").get_parameter_value().get<int>();

  try {
    // Attempt to connect the serial client
    // We don't expose the timeout to the user API to avoid usability concerns
    client_.ConnectClient(address);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());  // NOLINT
    return;
  }

  // Disable any previous heartbeat configurations
  DisableHeartbeat();

  // Configure the new heartbeat request
  EnableHeartbeat(state_freq);

  last_heartbeat_ = std::chrono::steady_clock::now();

  // Setup a timer to make sure that we have an active connection with the arm
  check_heartbeat_timer_ =
    this->create_wall_timer(1000ms, std::bind(&Driver::MonitorHeartbeatCb, this));

  // Connect packet callbacks
  client_.RegisterCallback(
    PacketId::kPosition, std::bind(&Driver::ProxyJointPositionCb, this, std::placeholders::_1));

  client_.RegisterCallback(
    PacketId::kVelocity, std::bind(&Driver::ProxyJointVelocityCb, this, std::placeholders::_1));

  client_.RegisterCallback(
    PacketId::kMode, std::bind(&Driver::ProxyModeCb, this, std::placeholders::_1));

  client_.RegisterCallback(
    PacketId::kMode, std::bind(&Driver::UpdateLastHeartbeatCb, this, std::placeholders::_1));
}

Driver::~Driver()
{
  DisableHeartbeat();
  client_.DisconnectClient();
}

void Driver::EnableHeartbeat(const int freq)
{
  // Specify that we want the position, velocity, and mode to be sent automatically
  const std::vector<unsigned char> heartbeat_config = {
    static_cast<unsigned char>(alpha_driver::PacketId::kPosition),
    static_cast<unsigned char>(alpha_driver::PacketId::kVelocity),
    static_cast<unsigned char>(alpha_driver::PacketId::kMode)};

  const Packet heartbeat_request(PacketId::kHeartbeatSet, DeviceId::kAllJoints, heartbeat_config);

  client_.Send(heartbeat_request);

  SetHeartbeatFreq(freq);
}

void Driver::SetHeartbeatFreq(const int freq)
{
  const std::vector<unsigned char> heartbeat_frequency = {static_cast<unsigned char>(freq)};
  const Packet request(PacketId::kHeartbeatFreqency, DeviceId::kAllJoints, heartbeat_frequency);
  client_.Send(request);
}

void Driver::DisableHeartbeat() { SetHeartbeatFreq(0); }

void Driver::ProxyJointPositionCb(const Packet & packet)
{ /* do cool things here */
}

void Driver::ProxyJointVelocityCb(const Packet & packet)
{ /* do cool things here */
}

void Driver::ProxyModeCb(const Packet & packet)
{ /* do cool things here */
}

void Driver::UpdateLastHeartbeatCb(const Packet &)
{
  last_heartbeat_ = std::chrono::steady_clock::now();
}

void Driver::MonitorHeartbeatCb()
{
  if (std::chrono::steady_clock::now() - last_heartbeat_ > 5s) {
    RCLCPP_WARN(  // NOLINT
      this->get_logger(),
      "Timeout occurred; the system has not received a heartbeat message in the last 5 seconds");
  }
}

}  // namespace alpha_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<alpha_driver::Driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
