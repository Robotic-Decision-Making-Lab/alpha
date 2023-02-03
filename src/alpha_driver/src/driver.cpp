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

#include <chrono>
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
  // Define the parameter descriptors for our ROS parameters
  auto address_desc = rcl_interfaces::msg::ParameterDescriptor{};
  address_desc.description = "Full path to the serial port file";

  auto timeout_desc = rcl_interfaces::msg::ParameterDescriptor{};
  timeout_desc.description =
    "Maximum time (ms) to wait for serial data before attempting a new read";

  // Declare ROS parameters
  this->declare_parameter("address", "", address_desc);
  this->declare_parameter("timeout", 500, timeout_desc);

  // Read our parameter values to use for the serial client
  const std::string address =
    this->get_parameter("address").get_parameter_value().get<std::string>();
  const int timeout = this->get_parameter("timeout").get_parameter_value().get<int>();

  // Attempt to connect the serial client
  client_.ConnectClient(address, timeout);

  // Connect a callback to republish received state data
  client_.RegisterCallback(
    PacketId::kPosition, std::bind(&Driver::PublishState, this, std::placeholders::_1));

  // Request the state from the manipulator at a frequency of 1hz
  request_state_timer_ = this->create_wall_timer(1000ms, std::bind(&Driver::RequestState, this));
}

void Driver::PublishState(const Packet & packet)
{
  for (auto data : packet.data()) {
    RCLCPP_INFO(this->get_logger(), "data: 0x%02hhx", data);
  }
}

void Driver::RequestState()
{
  const std::vector<unsigned char> data = {
    static_cast<unsigned char>(alpha_driver::PacketId::kPosition)};

  const Packet packet(PacketId::kRequest, DeviceId::kLinearJaws, data);

  client_.Send(packet);
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
