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

#include <chrono>
#include <limits>

#include "alpha_driver/device_id.hpp"
#include "alpha_driver/mode.hpp"
#include "alpha_driver/packet_id.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace alpha_hardware
{

hardware_interface::CallbackReturn AlphaHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("AlphaHardware"),
    "Initializing AlphaHardware system interface for ros2_control.");

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(  // NOLINT
      rclcpp::get_logger("AlphaHardware"),
      "Failed to initialize the AlphaHardware system interface.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_modes_.resize(
    info_.joints.size(),
    ControlMode::kVelocity);  // TODO(evan_palmer): make the default mode a parameter

  // Load ROS params
  serial_port_ = info_.hardware_parameters["serial_port"];
  heartbeat_timeout_ = std::stoi(info_.hardware_parameters["heartbeat_timeout"]);
  state_update_freq_ = std::stoi(info_.hardware_parameters["state_update_frequency"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // AlphaHardware has two command interfaces (position & velocity) and two state interfaces
    // (position & velocity)
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(  // NOLINT
        rclcpp::get_logger("AlphaHardware"),
        "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
        joint.command_interfaces.size());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_FATAL(  // NOLINT
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has a %s command interface. Expected %s or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(  // NOLINT
        rclcpp::get_logger("AlphaHardware"), "Joint '%s' has %zu state interfaces. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_FATAL(  // NOLINT
        rclcpp::get_logger("RRBotSystemMultiInterfaceHardware"),
        "Joint '%s' has a %s state interface. Expected %s or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("AlphaHardware"),
    "Successfully initialized the AlphaHardware system interface!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlphaHardware::on_configure(const rclcpp_lifecycle::State &)
{
  // Start the driver
  try {
    driver_.start(serial_port_, heartbeat_timeout_);
  }
  catch (const std::exception & e) {
    RCLCPP_FATAL(  // NOLINT
      rclcpp::get_logger("AlphaHardware"),
      "Failed to configure the serial driver for the AlphaHardware system interface.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  // Register callbacks for joint states
  driver_.subscribe(
    alpha_driver::PacketId::kPosition,
    std::bind(&AlphaHardware::update_position_cb, this, std::placeholders::_1));

  driver_.subscribe(
    alpha_driver::PacketId::kVelocity,
    std::bind(&AlphaHardware::update_velocity_cb, this, std::placeholders::_1));

  // Start a thread to request state updates
  running_.store(true);
  state_request_worker_ = std::thread(&AlphaHardware::poll_state, this, state_update_freq_);

  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("AlphaHardware"),
    "Successfully configured the AlphaHardware system interface for serial communication!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlphaHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(  // NOLINT
    rclcpp::get_logger("AlphaHardware"), "Shutting down the AlphaHardware system interface.");

  running_.store(false);
  state_request_worker_.join();
  driver_.stop();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AlphaHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Prepare for new command modes
  std::vector<ControlMode> new_modes = {};

  for (const std::string & key : start_interfaces) {
    for (auto & joint : info_.joints) {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION) {
        new_modes.push_back(ControlMode::kPosition);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_modes.push_back(ControlMode::kVelocity);
      }
    }
  }

  // Stop motion on all relevant joints that are stopping
  for (const std::string & key : stop_interfaces) {
    for (std::size_t i = 0; i < info_.joints.size(); i++) {
      if (key.find(info_.joints[i].name) != std::string::npos) {
        hw_commands_velocities_[i] = 0;
        control_modes_[i] = ControlMode::kUndefined;  // Revert to undefined
      }
    }
  }

  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    if (control_modes_[i] != ControlMode::kUndefined) {
      return hardware_interface::return_type::ERROR;
    }
    control_modes_[i] = new_modes[i];
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> AlphaHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AlphaHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AlphaHardware::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    driver_.set_mode(alpha_driver::Mode::kStandby, alpha_driver::DeviceId::kAllJoints);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("AlphaHardware"), e.what());  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AlphaHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    driver_.set_mode(alpha_driver::Mode::kDisable, alpha_driver::DeviceId::kAllJoints);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("AlphaHardware"), e.what());  // NOLINT
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AlphaHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Get access to the real-time states
  const std::lock_guard<std::mutex> lock(access_async_states_);

  // Copy the latest readings to the read ros2_control state interfaces
  // We need to maintain two state vectors here because the ros2_control endpoint won't have access
  // to the mutex needed to read the real-time states
  std::copy(
    hw_states_positions_.begin(), hw_states_positions_.end(), async_states_positions_.begin());
  std::copy(
    hw_states_velocities_.begin(), hw_states_velocities_.end(), async_states_velocities_.begin());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlphaHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Send the commands for each joint
  for (std::size_t i = 0; i < control_modes_.size(); i++) {
    switch (control_modes_[i]) {
      case ControlMode::kPosition:
        driver_.set_position(hw_commands_positions_[i], static_cast<alpha_driver::DeviceId>(i));
        break;
      case ControlMode::kVelocity:
        driver_.set_velocity(hw_commands_velocities_[i], static_cast<alpha_driver::DeviceId>(i));
        break;
      default:
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

void AlphaHardware::update_position_cb(const alpha_driver::Packet & packet)
{
  if (packet.data().size() != 4) {
    return;
  }

  float position;
  std::memcpy(&position, &packet.data()[0], sizeof(position));  // NOLINT

  const std::lock_guard<std::mutex> lock(access_async_states_);

  // We assume that the device ID is the index within the vector
  async_states_positions_[static_cast<std::size_t>(packet.device_id())] = position;
}

void AlphaHardware::update_velocity_cb(const alpha_driver::Packet & packet)
{
  if (packet.data().size() != 4) {
    return;
  }

  float velocity;
  std::memcpy(&velocity, &packet.data()[0], sizeof(velocity));  // NOLINT

  const std::lock_guard<std::mutex> lock(access_async_states_);

  // We assume that the device ID is the index within the vector
  async_states_velocities_[static_cast<std::size_t>(packet.device_id())] = velocity;
}

void AlphaHardware::poll_state(const int freq) const
{
  // Request position and velocity state information
  std::vector<alpha_driver::PacketId> packets = {
    alpha_driver::PacketId::kPosition, alpha_driver::PacketId::kVelocity};

  while (running_.load()) {
    driver_.request(packets, alpha_driver::DeviceId::kAllJoints);
    std::this_thread::sleep_for(std::chrono::seconds(1 / freq));
  }
}

}  // namespace alpha_hardware
