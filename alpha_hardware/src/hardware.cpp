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

#include "alpha_hardware/hardware.hpp"

#include <limits>

namespace alpha_hardware
{

hardware_interface::CallbackReturn AlphaHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Load ROS params
  serial_port_ = std::stod(info_.hardware_parameters["serial_port"]);
  heartbeat_timeout_ = std::stod(info_.hardware_parameters["heartbeat_timeout"]);
  state_update_freq_ = std::stod(info_.hardware_parameters["state_update_frequency"]);

  // Start the driver
  if (!driver_.start(serial_port_, heartbeat_timeout_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO (evan-palmer): register callbacks for joint states
  // TODO (evan-palmer): Start a thread to request state updates

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlphaHardware::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  /* data */
}

std::vector<hardware_interface::StateInterface> AlphaHardware::export_state_interfaces()
{ /* data */
}

std::vector<hardware_interface::CommandInterface> AlphaHardware::export_command_interfaces()
{ /* data */
}

hardware_interface::CallbackReturn AlphaHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  /* data */
}

hardware_interface::CallbackReturn AlphaHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  /* data */
}

hardware_interface::return_type AlphaHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  /* data */
}

hardware_interface::return_type AlphaHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  /* data */
}

}  // namespace alpha_hardware
