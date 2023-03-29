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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "alpha_driver/driver.hpp"
#include "alpha_driver/packet.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace alpha::hardware
{

class AlphaHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AlphaHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class ControlMode
  {
    kVelocity,
    kPosition,
  };

  /**
   * @brief Write the current position of the robot received from the serial client to the
   * respective asynchronous vector.
   *
   * @param packet The position packet that signaled the callback.
   */
  void updatePositionCb(const alpha::driver::Packet & packet);

  /**
   * @brief Write the current velocity of the robot received from the serial client to the
   * respective asynchronous vector.
   *
   * @param packet The velocity packet that signaled the callback.
   */
  void updateVelocityCb(const alpha::driver::Packet & packet);

  /**
   * @brief Asynchronously read the current state of the robot by polling the robot serial
   * interface.
   *
   * @param freq The frequency (Hz) that the interface should poll the current robot state at.
   */
  void pollState(int freq) const;

  // Driver things
  alpha::driver::Driver driver_;
  std::thread state_request_worker_;
  std::atomic<bool> running_{false};

  // ROS parameters
  std::string serial_port_;
  int state_update_freq_;

  // ros2_control command interfaces
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_positions_;

  // ros2_control state interfaces
  std::vector<double> hw_states_positions_, async_states_positions_;
  std::vector<double> hw_states_velocities_, async_states_velocities_;
  std::vector<ControlMode> control_modes_;

  std::mutex access_async_states_;
};

}  // namespace alpha::hardware
