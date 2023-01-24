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

#include "alpha_driver/serial_client.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>

#include "alpha_driver/packet.hpp"

namespace alpha_driver
{

SerialClient::SerialClient(const std::string & device, const int baudrate)
{
  if (device.empty()) {
    throw std::runtime_error("Attempted to open file for unassigned file path.");
  }

  // Open the serial port
  fd_ = open(device.c_str(), O_RDWR);

  if (fd_ == -1) {
    throw std::runtime_error(
      "Could not open specified serial device. Please verify that the device name is correct and "
      "that you have read/write permissions");
  }

  // Configure the linux file for serial communication
  struct termios tty;

  // Get the existing settings
  if (tcgetattr(fd_, &tty) < 0) {
    throw std::runtime_error("Unable to get the current terminal configurations.");
  }

  // TODO: Configure termios settings

  running_ = Status::kRunning;

  // TODO: Start a new thread to receive data
}

}  // namespace alpha_driver
