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
   * @brief Represents the current state of the serial client. Can be either running or stopped.
   *
   */
  enum class Status
  {
    kRunning,
    kStopped,
  };

  /**
   * @brief Construct a new Serial Client object
   *
   * @param device serial port to connect to
   * @param baudrate baudrate of the serial connection
   */
  SerialClient(const std::string & device, const int baudrate);

  /**
   * @brief Destroy the Serial Client object. This shuts down all threads.
   *
   */
  ~SerialClient();

  /**
   * @brief Send a packet over the serial connection.
   *
   * @param packet message to send to the Reach Alpha manipulator
   */
  void Send(const Packet & packet);

  /**
   * @brief Register a new callback function to a specified packet type.
   *
   * @param packet_type type of packet that the callback should be signaled on
   * @param callback function that should be executed when a message of a given type is received
   */
  void Receive(const PacketId packet_type, std::function<void(Packet)> callback);

private:
  // Map used to store the callback functions for messages. Keys should be the ID for a message,
  // and the values are the callback functions to execute when a message is received.
  std::unordered_map<PacketId, std::vector<std::function<void(Packet)>>> callbacks_;

  // Serial port pointer
  int fd_;

  // Flag indicating whether the serial client is running
  Status running_;

  // Thread responsible for receiving incoming data and executing the respective callbacks
  std::thread rx_worker_;

  // Incoming data buffer
  std::vector<unsigned char> buffer_;
};

}  // namespace alpha_driver
