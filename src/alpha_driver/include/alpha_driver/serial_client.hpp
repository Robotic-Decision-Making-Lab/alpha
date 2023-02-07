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
#include <functional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "alpha_driver/packet.hpp"
#include "alpha_driver/packet_id.hpp"

namespace alpha_driver
{

class SerialClient
{
public:
  /**
   * @brief Attempt to establish a connection with the Alpha manipulator.
   *
   * @note This configures the serial port for R/W operation at a baudrate of 115200.
   *
   * @param device full path to the serial device file
   * @param polling_timeout_ms timeout (ms) between serial port reads; used for VTIME
   */
  void connect(const std::string & device, const int polling_timeout_ms = 500);

  /**
   * @brief Shutdown the serial client.
   */
  void disconnect();

  /**
   * @brief Send a packet over the serial connection.
   *
   * @note This method performs packet encoding. It is not necessary to encode the data before
   * calling this method.
   *
   * @param packet message to send to the Reach Alpha manipulator; the data should not yet be
   * encoded.
   */
  void send(const Packet & packet) const;

  /**
   * @brief Register a new callback function for a specified packet type.
   *
   * @param packet_type type of packet that the callback should be registered to
   * @param callback function that should be executed when a message of a given type is received
   */
  void register_callback(PacketId packet_type, const std::function<void(Packet)> & callback);

  /**
   * @brief Indicates whether or not the serial client is currently active.
   *
   * @note To be considered 'active' there must be an open serial connection and a worker should be
   * polling the RX.
   *
   * @return true
   * @return false
   */
  bool active() const;  // NOLINT

private:
  /**
   * @brief Current state of the serial port.
   */
  enum class PortState
  {
    kOpen,   // The serial port is currently open
    kClosed  // The serial port is currently closed
  };

  /**
   * @brief Poll the serial line for incoming data.
   *
   * @note This method is executed by the RX thread. Furthermore, this method is
   * a blocking method that runs indefinitely. The main loop is terminated by
   * the atomic @ref running_ flag.
   */
  void poll() const;

  /**
   * @brief Map used to store the callback functions for messages.
   *
   * @note Keys should be the ID of a message and the values are the callback functions to execute
   * when a message the respective packet ID is received.
   */
  std::unordered_map<PacketId, std::vector<std::function<void(Packet)>>> callbacks_;

  // Serial port file descriptor
  int handle_;

  std::atomic<bool> running_{false};  // This flag is used to control the RX main loop
  PortState port_status_ = PortState::kClosed;

  // Thread responsible for receiving incoming data and executing the respective callbacks
  std::thread rx_worker_;
};

}  // namespace alpha_driver
