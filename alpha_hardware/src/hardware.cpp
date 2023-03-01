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
#include <cmath>
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
    RCLCPP_FATAL(  // NOLINT
      rclcpp::get_logger("AlphaHardware"),
      "Failed to initialize the AlphaHardware system interface.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  async_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  async_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // We set the default mode to position, but this will get changed as soon as the controller is
  // loaded
  control_modes_.resize(info_.joints.size(), ControlMode::kPosition);

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
        rclcpp::get_logger("AlphaHardware"),
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
        rclcpp::get_logger("AlphaHardware"),
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
  const std::vector<std::string> & /* stop interfaces */)
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

  // Make sure that command modes are set to all interfaces at the same time
  if (!new_modes.empty() && new_modes.size() != info_.joints.size()) {
    return hardware_interface::return_type::ERROR;
  }

  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    control_modes_[i] = new_modes[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlphaHardware::perform_command_mode_switch(
  const std::vector<std::string> & /* start interfaces */,
  const std::vector<std::string> & /* stop interfaces */)
{
  // The Alpha arm takes care of most of the mode switching for us. To make things a bit safer
  // though, we stop the robot before switching command modes
  driver_.set_velocity(0, alpha_driver::DeviceId::kAllJoints);

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
  hw_states_positions_ = async_states_positions_;
  hw_states_velocities_ = async_states_velocities_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlphaHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Send the commands for each joint
  for (std::size_t i = 0; i < control_modes_.size(); i++) {
    switch (control_modes_[i]) {
      case ControlMode::kPosition:
        if (!std::isnan(hw_commands_positions_[i])) {
          // Get the target device
          const auto target_device = static_cast<alpha_driver::DeviceId>(i + 1);

          // Get the target position; if the command is for the jaws, then convert from m to mm
          const double target_position =
            static_cast<alpha_driver::DeviceId>(i + 1) == alpha_driver::DeviceId::kLinearJaws
              ? hw_commands_positions_[i] * 1000
              : hw_commands_positions_[i];
          driver_.set_position(target_position, target_device);
        }
        break;
      case ControlMode::kVelocity:
        if (!std::isnan(hw_commands_velocities_[i])) {
          // Get the target device
          const auto target_device = static_cast<alpha_driver::DeviceId>(i + 1);

          // Get the target velocity; if the command is for the jaws, then convert from m/s to mm/s
          const double target_velocity =
            static_cast<alpha_driver::DeviceId>(i + 1) == alpha_driver::DeviceId::kLinearJaws
              ? hw_commands_velocities_[i] * 1000
              : hw_commands_velocities_[i];

          driver_.set_velocity(target_velocity, target_device);
        }
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

  // Convert from mm to m if the message is from the jaws
  if (packet.device_id() == alpha_driver::DeviceId::kLinearJaws) {
    position /= 1000;
  }

  const std::lock_guard<std::mutex> lock(access_async_states_);

  // We assume that the device ID is the index within the vector
  async_states_positions_[static_cast<std::size_t>(packet.device_id()) - 1] = position;
}

void AlphaHardware::update_velocity_cb(const alpha_driver::Packet & packet)
{
  if (packet.data().size() != 4) {
    return;
  }

  float velocity;
  std::memcpy(&velocity, &packet.data()[0], sizeof(velocity));  // NOLINT

  // Convert from mm/s to m/s if the message is from the jaws
  if (packet.device_id() == alpha_driver::DeviceId::kLinearJaws) {
    velocity /= 1000;
  }

  const std::lock_guard<std::mutex> lock(access_async_states_);

  // We assume that the device ID is the index within the vector
  async_states_velocities_[static_cast<std::size_t>(packet.device_id()) - 1] = velocity;
}

void AlphaHardware::poll_state(const int freq) const
{
  while (running_.load()) {
    // There are a few important things to note here:
    //   1. Yes, we could use the kAllJoints device ID, but for some reason, the response rate for
    //      each joints becomes less reliable when we use kAllJoints. We get better response rates
    //      from the joints when we split this up.
    //   2. Yes, we could also create a request with multiple packet IDs, but, again, there are
    //      bugs in the serial communication when that is used. Specifically, data is more likely
    //      to become corrupted, resulting in bad reads. So instead we just request them separately.
    driver_.request(alpha_driver::PacketId::kVelocity, alpha_driver::DeviceId::kLinearJaws);
    driver_.request(alpha_driver::PacketId::kVelocity, alpha_driver::DeviceId::kRotateEndEffector);
    driver_.request(alpha_driver::PacketId::kVelocity, alpha_driver::DeviceId::kBendElbow);
    driver_.request(alpha_driver::PacketId::kVelocity, alpha_driver::DeviceId::kBendShoulder);
    driver_.request(alpha_driver::PacketId::kVelocity, alpha_driver::DeviceId::kRotateBase);

    driver_.request(alpha_driver::PacketId::kPosition, alpha_driver::DeviceId::kLinearJaws);
    driver_.request(alpha_driver::PacketId::kPosition, alpha_driver::DeviceId::kRotateEndEffector);
    driver_.request(alpha_driver::PacketId::kPosition, alpha_driver::DeviceId::kBendElbow);
    driver_.request(alpha_driver::PacketId::kPosition, alpha_driver::DeviceId::kBendShoulder);
    driver_.request(alpha_driver::PacketId::kPosition, alpha_driver::DeviceId::kRotateBase);

    std::this_thread::sleep_for(std::chrono::seconds(1 / freq));
  }
}

}  // namespace alpha_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(alpha_hardware::AlphaHardware, hardware_interface::SystemInterface)
