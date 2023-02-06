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

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "alpha_driver/mode.hpp"
#include "alpha_driver/packet.hpp"
#include "alpha_driver/packet_id.hpp"
#include "alpha_driver/serial_client.hpp"

namespace alpha_driver
{

class Driver
{
public:
  Driver();

  bool start(const std::string & serial_port, const int heartbeat_timeout_ms = 3000);

  void stop();

  void set_mode(Mode mode, DeviceId device) const;

  void set_velocity(float velocity, DeviceId device) const;

  void set_position(float position, DeviceId device) const;

  void set_relative_position(float relative_position, DeviceId device) const;

  void set_indexed_position(float indexed_position, DeviceId device) const;

  void set_current(float current, DeviceId device) const;

  void request(PacketId packet_type, DeviceId device) const;

  void subscribe(PacketId packet_type, const std::function<void(Packet)> & callback);

private:
  void send_float(float value, PacketId packet_type, DeviceId device) const;

  void enable_heartbeat(const int freq);

  void disable_heartbeat();

  void set_heartbeat_freq(const int freq);

  void update_last_heartbeat_cb(const Packet & packet);

  void monitor_heartbeat(const int heartbeat_timeout_ms) const;

  SerialClient client_;

  std::thread heartbeat_worker_;
  std::atomic<bool> running_{false};
  mutable std::mutex last_heartbeat_lock_;

  std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_;
};

}  // namespace alpha_driver
