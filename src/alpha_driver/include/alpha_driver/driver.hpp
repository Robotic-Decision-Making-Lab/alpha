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

#include <chrono>

#include "alpha_driver/packet.hpp"
#include "alpha_driver/serial_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace alpha_driver
{

class Driver : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Driver object.
   */
  Driver();

  /**
   * @brief Destroy the Driver object.
   *
   * @note Overrides the default destructor to shut down the serial client.
   */
  ~Driver();

private:
  /**
   * @brief Enable heartbeat messages from the Alpha manipulator.
   *
   * @note Heartbeat messages are defined by the BPL protocol simply as messages that are
   * automatically sent by the manipulator - not as a unique packet type. The messages requested for
   * automatic sending include velocity, position, and mode messages. These messages are sent by
   * all devices.
   *
   * @param freq frequency that the heartbeat messages should be sent at
   */
  void EnableHeartbeat(const int freq);

  /**
   * @brief Set the frequency that heartbeat messages are sent at.
   *
   * @note Heartbeat messages can be sent at frequencies in the range [0, 255]. If the frequency is
   * set to 0, then the heartbeat will be disabled.
   *
   * @param freq
   */
  void SetHeartbeatFreq(const int freq);

  /**
   * @brief Disable heartbeat messages.
   *
   * @remark This is implemented for usability purposes and simply calls the @ref SetHeartbeatFreq
   * with a frequency of 0.
   */
  void DisableHeartbeat();

  void ProxyJointPositionCb(const Packet & packet);
  void ProxyJointVelocityCb(const Packet & packet);
  void ProxyModeCb(const Packet & packet);

  /**
   * @brief Update the @ref last_heartbeat_ timestamp
   *
   * @param packet heartbeat packet
   */
  void UpdateLastHeartbeatCb(const Packet & packet);

  /**
   * @brief Verify that a heartbeat packet has been received in the last 5 seconds
   *
   * @note If a heartbeat has occurred, it is the user's responsibility to handle the timeout. This
   * method is not intended to act as a watchdog.
   */
  void MonitorHeartbeatCb();

  SerialClient client_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;

  rclcpp::TimerBase::SharedPtr check_heartbeat_timer_;
  std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_;
};

}  // namespace alpha_driver
