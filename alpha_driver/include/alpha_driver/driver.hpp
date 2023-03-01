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
#include <string>
#include <thread>
#include <vector>

#include "alpha_driver/mode.hpp"
#include "alpha_driver/packet.hpp"
#include "alpha_driver/packet_id.hpp"
#include "alpha_driver/serial_client.hpp"

namespace alpha_driver
{

class Driver
{
public:
  /**
   * @brief Construct a new Driver object.
   */
  Driver();

  /**
   * @brief Start the driver.
   *
   * @note This will attempt to connect the serial client and start the heartbeat.
   *
   * @param serial_port serial port that the manipulator is available at
   * @param heartbeat_timeout maximum time (s) between heartbeat messages before the connection is
   * considered timed out; must be greater than 1, defaults to 3 seconds
   */
  void start(const std::string & serial_port, int heartbeat_timeout = 3);

  /**
   * @brief Stop the driver.
   *
   * @note This will stop the heartbeat and disconnect the serial client.
   */
  void stop();

  /**
   * @brief Set the mode of a device.
   *
   * @param mode desired operating mode
   * @param device device whose mode should be configured
   */
  void set_mode(Mode mode, DeviceId device) const;

  /**
   * @brief Set the velocity of a device.
   *
   * @note If the velocity setpoint is set above the velocity limit, the velocity will be set to the
   * limit.
   *
   * @param velocity velocity setpoint; for rotational devices, the velocity should be provided in
   * rad/s. For linear devices, the velocity should be provided in mm/s.
   * @param device device whose velocity should be set
   */
  void set_velocity(float velocity, DeviceId device) const;

  /**
   * @brief Set the position of a device.
   *
   * @note If the position setpoint is outside of the configured limits, the command is ignored.
   *
   * @param position position setpoint; for rotational devices, this should be an angle in the range
   * [0, 2pi]. For linear divices, the velocity should be a distance in mm.
   * @param device device whose position should be set
   */
  void set_position(float position, DeviceId device) const;

  /**
   * @brief Set the relative position of a device.
   *
   * @note The actuator will move from its current position by the amount specified.
   *
   * @param relative_position relative position setpoint; for rotational devices, this should be an
   * angle in the range [0, 2pi]. For linear divices, the velocity should be a distance in mm.
   * @param device device whose relative position should be set.
   */
  void set_relative_position(float relative_position, DeviceId device) const;

  /**
   * @brief Set the current setpoint of the motor windings.
   *
   * @note Demanding current that is out of range will set the current to its maximum.
   *
   * @param current current setpoint (mAh)
   * @param device device whose current should be set
   */
  void set_current(float current, DeviceId device) const;

  /**
   * @brief Request a packet from the Alpha manipulator.
   *
   * @remark This intended to be used in conjunction with the @ref subscribe method.
   *
   * @param packet_type packet type that the manipulator should send
   * @param device device that should sent the packet
   */
  void request(PacketId packet_type, DeviceId device) const;

  /**
   * @brief Request multiple packets from the Alpha manipulator.
   *
   * @note Up to to 10 packets may be requested at once
   *
   * @param packet_types vector of packet types that the manipulator should send
   * @param device device that should sent the packets
   */
  void request(std::vector<PacketId> & packet_types, DeviceId device) const;

  /**
   * @brief Register a callback function to be executed when a packet of the specified type is
   * received from the manipulator.
   *
   * @note This is intended to be used in conjunction with the @ref request method.
   *
   * @param packet_type packet type that should signal this function
   * @param callback function to execution when a packet of the given type is received from the
   * manipulator
   */
  void subscribe(PacketId packet_type, const std::function<void(Packet)> & callback);

private:
  /**
   * @brief Send a float message.
   *
   * @note This is a helper method used to cast a float value to a vector of unsigned chars and
   * send the resulting data.
   *
   * @param value float value to send
   * @param packet_type type of packet to construct
   * @param device device that should receive the message
   */
  void send_float(float value, PacketId packet_type, DeviceId device) const;

  /**
   * @brief Enable heartbeat messages from the Alpha manipulator.
   *
   * @note There isn't a single dedicated message available for heartbeat messages. Because of this,
   * we specify the Model Number packet as the heartbeat message.
   *
   * @remark While a more helpful message could be request as the heartbeat (e.g., position,
   * velocity, mode, etc.), the interface would become less usable and a bit more confusing if some
   * state messages are broadcasted but others aren't. Furthermore, those messages may not be
   * desired by users. Therefore, we leave it up to the users to request state information.
   *
   * @param freq frequency that the heartbeats should be sent at
   */
  void enable_heartbeat(int freq);

  /**
   * @brief Disable heartbeat messages from the Alpha manipulator.
   *
   * @note This is equivalent to setting the heartbeat frequency to 0.
   */
  void disable_heartbeat();

  /**
   * @brief Set the frequency that heartbeat messages should be sent.
   *
   * @param freq frequency that the heartbeat messages should be sent at
   */
  void set_heartbeat_freq(int freq);

  /**
   * @brief Update the timestamp to indicate that a heartbeat was received.
   */
  void update_last_heartbeat_cb(const Packet &);

  /**
   * @brief Monitor the latest heartbeat timestamp to determine whether or not the connection has
   * timed out.
   *
   * @note This method is not designed to be a watchdog. If a timeout occurs, it is the user's
   * responsibility to respond.
   *
   * @param heartbeat_timeout_ms maximum allowable time between heartbeat messages before notifying
   * users that a timeout may have occurred.
   */
  void monitor_heartbeat(int heartbeat_timeout_ms) const;

  SerialClient client_;

  std::thread heartbeat_worker_;
  std::atomic<bool> running_{false};
  mutable std::mutex last_heartbeat_lock_;

  std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_;
};

}  // namespace alpha_driver
