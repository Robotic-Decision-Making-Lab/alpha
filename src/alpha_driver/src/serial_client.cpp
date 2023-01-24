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
#include <iostream>
#include <stdexcept>
#include <vector>

#include "alpha_driver/packet.hpp"

namespace alpha_driver
{

SerialClient::SerialClient(const std::string & device, const int timeout_ms, const bool blocking)
: running_(false),
  port_status_(PortState::kOpen),
  heartbeat_status_(HeartbeatState::kDead)
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

  // Set the serial port status to open
  port_status_ = PortState::kOpen;

  // Configure the linux file for serial communication
  struct termios tty;

  // Get the existing settings
  if (tcgetattr(fd_, &tty) < 0) {
    throw std::runtime_error("Unable to get the current terminal configurations.");
  }

  // Set the baudrate to 115200 as defined by the Alpha Reach specification
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);  // Enable the receiver and set the mode to local mode
  tty.c_cflag &= ~CSIZE;            // Mask the character size bits
  tty.c_cflag |= CS8;               // Use 8 data bits per byte
  tty.c_cflag &= ~PARENB;           // Disable parity
  tty.c_cflag &= ~CSTOPB;           // Only one stop bit is used
  tty.c_cflag &= ~CRTSCTS;          // Disable RTS/CTS hardware flow control

  // These settings are obtained from the following source:
  tty.c_lflag &= ~ICANON;                  // Disable canonical input
  tty.c_lflag &= ~ECHO;                    // Disable echo
  tty.c_lflag &= ~ECHOE;                   // Disable erasure
  tty.c_lflag &= ~ECHONL;                  // Disable new-line echo
  tty.c_lflag &= ~ISIG;                    // Disable signals
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off io control
  tty.c_iflag &=
    ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
      ICRNL);  // Disable special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Disable output post-processing
  tty.c_oflag &= ~ONLCR;  // Disable conversion of newline to carriage return

  tty.c_cc[VTIME] = static_cast<cc_t>(timeout_ms / 100);  // Set the timeout
  tty.c_cc[VMIN] = blocking ? 1 : 0;                      // Block until data is received

  // Save the configurations
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("Unable to save the terminal configurations.");
  }

  // TODO(evan-palmer): Register a callback to handle the heartbeat status for the arm

  // TODO(evan-palmer): Start a new thread to receive data
}

auto SerialClient::Send(const Packet & packet) const -> void
{
  if (port_status_ != PortState::kOpen) {
    throw std::runtime_error("Cannot send messages without first opening the serial port.");
  }

  std::vector<unsigned char> encoded_data = packet.Encode();

  if (write(fd_, encoded_data.data(), encoded_data.size()) < 0) {
    // TODO(evan-palmer): integrate GLOG instead of using cout
    std::cout << "An error occurred when attempting to write a message." << std::endl;
  }
}

auto SerialClient::Receive(PacketId packet_type, const std::function<void(Packet)> & callback)
  -> void
{
  // Register the callback
  callbacks_[packet_type].push_back(callback);
}

auto SerialClient::active() const -> bool
{
  return (
    running_ && port_status_ == PortState::kOpen && heartbeat_status_ == HeartbeatState::kBeating);
}

auto SerialClient::Read() -> void
{
  running_ = true;

  while (running_.load()) {
    /* do stuff */
  }
}

}  // namespace alpha_driver
