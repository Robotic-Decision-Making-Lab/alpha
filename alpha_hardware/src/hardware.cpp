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

namespace alpha_hardware
{

hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info)
{
  /* data */
}

hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state)
{
  /* data */
}

hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  /* data */
}

std::vector<hardware_interface::StateInterface> export_state_interfaces() { /* data */ }

std::vector<hardware_interface::CommandInterface> export_command_interfaces() { /* data */ }

hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
{
  /* data */
}

hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  /* data */
}

hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  /* data */
}

hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  /* data */
}

}  // namespace alpha_hardware
