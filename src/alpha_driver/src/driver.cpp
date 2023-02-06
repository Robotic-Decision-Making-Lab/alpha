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
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace alpha_driver
{

Driver::Driver()
{
  // Connect packet callbacks
  client_.register_callback(
    PacketId::kPosition, std::bind(&Driver::proxy_joint_position_cb, this, std::placeholders::_1));

  client_.register_callback(
    PacketId::kVelocity, std::bind(&Driver::proxy_joint_velocity_cb, this, std::placeholders::_1));

  client_.register_callback(
    PacketId::kMode, std::bind(&Driver::update_last_heartbeat_cb, this, std::placeholders::_1));
}

bool Driver::start(
  const std::string & serial_port, const int state_update_freq, const int polling_timeout)
{
  try {
    // Attempt to connect the serial client
    // We don't expose the timeout to the user API to avoid usability concerns
    client_.connect(serial_port, polling_timeout);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("AlphaDriver"), e.what());  // NOLINT
    return false;
  }

  // Disable any previous heartbeat configurations
  disable_heartbeat();

  // Configure the new heartbeat request
  enable_heartbeat(state_update_freq);

  last_heartbeat_ = std::chrono::steady_clock::now();

  // TODO (evan-palmer): spawn a new thread that checks the heartbeat status
  // TODO (evan-palmer): add a mutex to restrict access to the `last_heartbeat_` variable

  return true;
}

void Driver::stop()
{
  disable_heartbeat();
  client_.disconnect();
}

void Driver::enable_heartbeat(const int freq)
{
  // Specify that we want the position, velocity, and mode to be sent automatically
  const std::vector<unsigned char> heartbeat_config = {
    static_cast<unsigned char>(alpha_driver::PacketId::kPosition),
    static_cast<unsigned char>(alpha_driver::PacketId::kVelocity),
    static_cast<unsigned char>(alpha_driver::PacketId::kMode)};

  const Packet heartbeat_request(PacketId::kHeartbeatSet, DeviceId::kAllJoints, heartbeat_config);

  client_.send(heartbeat_request);

  set_heartbeat_freq(freq);
}

void Driver::set_heartbeat_freq(const int freq)
{
  const std::vector<unsigned char> heartbeat_frequency = {static_cast<unsigned char>(freq)};
  const Packet request(PacketId::kHeartbeatFreqency, DeviceId::kAllJoints, heartbeat_frequency);
  client_.send(request);
}

void Driver::disable_heartbeat() { set_heartbeat_freq(0); }

void Driver::proxy_joint_position_cb(const Packet & packet)
{
  if (packet.data().size() != 4) {
    return;
  }

  float position;
  std::memcpy(&position, &packet.data()[0], sizeof(position));  // NOLINT
}

void Driver::proxy_joint_velocity_cb(const Packet & packet)
{
  if (packet.data().size() != 4) {
    return;
  }

  float velocity;
  std::memcpy(&velocity, &packet.data()[0], sizeof(velocity));  // NOLINT
}

void Driver::update_last_heartbeat_cb(const Packet &)
{
  last_heartbeat_ = std::chrono::steady_clock::now();
}

void Driver::monitor_heartbeat_cb()
{
  if (std::chrono::steady_clock::now() - last_heartbeat_ > 5s) {
    RCLCPP_WARN(  // NOLINT
      rclcpp::get_logger("AlphaDriver"),
      "Timeout occurred; the system has not received a heartbeat message in the last 5 seconds");
  }
}

}  // namespace alpha_driver
