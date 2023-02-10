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
  client_.register_callback(
    PacketId::kModelNumber,
    std::bind(&Driver::update_last_heartbeat_cb, this, std::placeholders::_1));
}

void Driver::start(const std::string & serial_port, int heartbeat_timeout_ms)
{
  // Attempt to connect the serial client
  // We don't expose the VTIME timeout to the user API to avoid usability concerns
  client_.connect(serial_port);

  // Disable any previous heartbeat configurations
  disable_heartbeat();

  // Configure the new heartbeat request
  // The heartbeat should be sent 3 times per timeout interval
  enable_heartbeat(3 / heartbeat_timeout_ms);

  {
    const std::lock_guard<std::mutex> lock(last_heartbeat_lock_);
    last_heartbeat_ = std::chrono::steady_clock::now();
  }

  running_.store(true);

  // Start the thread that monitors heartbeats from the manipulator
  heartbeat_worker_ = std::thread(&Driver::monitor_heartbeat, this, heartbeat_timeout_ms);
}

void Driver::stop()
{
  disable_heartbeat();
  running_.store(false);
  heartbeat_worker_.join();
  client_.disconnect();
}

void Driver::set_mode(Mode mode, DeviceId device) const
{
  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a message to the manipulator without an active connection!");
  }

  const std::vector<unsigned char> mode_setting = {static_cast<unsigned char>(mode)};

  const Packet packet(PacketId::kMode, device, mode_setting);

  client_.send(packet);
}

void Driver::set_velocity(float velocity, DeviceId device) const
{
  send_float(velocity, PacketId::kVelocity, device);
}

void Driver::set_position(float position, DeviceId device) const
{
  send_float(position, PacketId::kPosition, device);
}

void Driver::set_relative_position(float relative_position, DeviceId device) const
{
  send_float(relative_position, PacketId::kRelativePosition, device);
}

void Driver::set_current(float current, DeviceId device) const
{
  send_float(current, PacketId::kCurrent, device);
}

void Driver::request(PacketId packet_type, DeviceId device) const
{
  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a request to the manipulator without an active connection!");
  }

  const std::vector<unsigned char> request_type = {static_cast<unsigned char>(packet_type)};

  const Packet packet(PacketId::kRequest, device, request_type);

  client_.send(packet);
}

void Driver::request(std::vector<PacketId> & packet_types, DeviceId device) const
{
  if (packet_types.size() > 10) {
    throw std::logic_error("Cannot request more than 10 packets from a device at once.");
  }

  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a request to the manipulator without an active connection!");
  }

  std::vector<unsigned char> request_types;
  request_types.reserve(packet_types.size());

  // Cast to unsigned char
  for (auto type : packet_types) {
    request_types.push_back(static_cast<unsigned char>(type));
  }

  const Packet packet(PacketId::kRequest, device, request_types);

  client_.send(packet);
}

void Driver::subscribe(PacketId packet_type, const std::function<void(Packet)> & callback)
{
  client_.register_callback(packet_type, callback);
}

void Driver::send_float(float value, PacketId packet_type, DeviceId device_id) const
{
  if (!running_.load() || !client_.active()) {
    throw std::runtime_error(
      "Cannot send a message to the manipulator without an active connection!");
  }

  // Convert the float into a vector of bytes
  std::vector<unsigned char> reinterpretted_vector;
  reinterpretted_vector.reserve(sizeof(value));
  std::memcpy(&reinterpretted_vector[0], &value, sizeof(value));  // NOLINT

  const Packet packet(packet_type, device_id, reinterpretted_vector);

  client_.send(packet);
}

void Driver::enable_heartbeat(const int freq)
{
  // We request the model number as the heartbeat because there isn't an official heartbeat message
  const std::vector<unsigned char> heartbeat_config = {
    static_cast<unsigned char>(alpha_driver::PacketId::kModelNumber)};

  const Packet packet(PacketId::kHeartbeatSet, DeviceId::kAllJoints, heartbeat_config);

  client_.send(packet);

  set_heartbeat_freq(freq);
}

void Driver::disable_heartbeat() { set_heartbeat_freq(0); }

void Driver::set_heartbeat_freq(const int freq)
{
  const std::vector<unsigned char> heartbeat_frequency = {static_cast<unsigned char>(freq)};
  const Packet packet(PacketId::kHeartbeatFreqency, DeviceId::kAllJoints, heartbeat_frequency);
  client_.send(packet);
}

void Driver::update_last_heartbeat_cb(const Packet &)
{
  const std::lock_guard<std::mutex> lock(last_heartbeat_lock_);

  last_heartbeat_ = std::chrono::steady_clock::now();
}

void Driver::monitor_heartbeat(const int heartbeat_timeout_ms) const
{
  while (running_.load()) {
    // Make sure that the lock is properly scoped so that we don't accidentally keep the look
    // forever
    {
      const std::lock_guard<std::mutex> lock(last_heartbeat_lock_);

      if (
        std::chrono::steady_clock::now() - last_heartbeat_ >
        std::chrono::milliseconds(heartbeat_timeout_ms)) {
        RCLCPP_WARN(  // NOLINT
          rclcpp::get_logger("AlphaDriver"),
          "Timeout occurred; the system has not received a heartbeat message in the last %d "
          "seconds",
          heartbeat_timeout_ms);
      }
    }

    // We don't need to check more often than our timeout requires us to
    std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_timeout_ms));
  }
}

}  // namespace alpha_driver
