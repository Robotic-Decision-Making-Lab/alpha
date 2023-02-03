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
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "alpha_driver/packet.hpp"

namespace alpha_driver
{

SerialClient::SerialClient(const rclcpp::Logger & logger)
: logger_(logger)
{
}

void SerialClient::ConnectClient(const std::string & device, int timeout_ms)
{
  if (device.empty()) {
    throw std::runtime_error("Attempted to open file using an unassigned file path.");
  }

  fd_ = open(device.c_str(), O_RDWR);

  if (fd_ == -1) {
    throw std::runtime_error(
      "Could not open specified serial device. Please verify that the device is not already being "
      "used and that you have configured read/write permissions.");
  }

  port_status_ = PortState::kOpen;

  struct termios tty;

  // Get the terminal existing settings
  if (tcgetattr(fd_, &tty) < 0) {
    throw std::runtime_error("Unable to get the current terminal configurations.");
  }

  // Set the baudrate to 115200 as defined by the Alpha Reach specification
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);         // Enable the receiver and set the mode to local mode
  tty.c_cflag &= ~CSIZE;                   // Mask the character size bits
  tty.c_cflag |= CS8;                      // Use 8 data bits per byte
  tty.c_cflag &= ~PARENB;                  // Disable parity
  tty.c_cflag &= ~CSTOPB;                  // Only one stop bit is used
  tty.c_cflag &= ~CRTSCTS;                 // Disable RTS/CTS hardware flow control
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

  tty.c_cc[VTIME] = static_cast<cc_t>(timeout_ms / 100);  // Set the timeout; convert to deciseconds
  tty.c_cc[VMIN] = 0;  // We can't set this in a thread otherwise it may block indefinitely

  // Save the configurations
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("Unable to save the terminal configurations.");
  }

  // Start reading data from the serial port
  rx_worker_ = std::thread(&SerialClient::Read, this);

  // Give the RX thread a few ms to start up
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Final check to make sure that the client was properly connected
  if (!active()) {
    throw std::runtime_error(
      "An error occurred while attempting to establish a serial connection.");
  }
}

void SerialClient::DisconnectClient()
{
  running_.store(false);
  rx_worker_.join();
  close(fd_);
}

void SerialClient::Send(const Packet & packet) const
{
  std::vector<unsigned char> encoded_data = packet.Encode();

  if (write(fd_, encoded_data.data(), encoded_data.size()) < 0) {
    RCLCPP_WARN(logger_, "An error occurred while attempting to write a message.");  // NOLINT
  }
}

void SerialClient::RegisterCallback(
  PacketId packet_type, const std::function<void(Packet)> & callback)
{
  callbacks_[packet_type].push_back(callback);
}

bool SerialClient::active() const { return (running_.load() && port_status_ == PortState::kOpen); }

void SerialClient::Read()
{
  running_.store(true);

  while (running_.load()) {
    buffer_.clear();

    // We need to make sure to resize each time
    buffer_.resize(256);

    const int size = read(fd_, buffer_.data(), buffer_.size());

    if (size < 0) {
      RCLCPP_WARN(  // NOLINT
        logger_, "An error occurred while attempting to read a message from the serial port.");
    }

    buffer_.resize(size);

    if (size > 0) {
      try {
        const Packet data = Packet::Decode(buffer_);

        for (auto & callback : callbacks_[data.packet_id()]) {
          callback(data);
        }
      }
      catch (const std::exception & e) {
        RCLCPP_WARN(logger_, e.what());  // NOLINT
      }
    }
  }
}

}  // namespace alpha_driver
