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
#include "rclcpp/rclcpp.hpp"

namespace alpha_driver
{

class SerialClient
{
public:
  /**
   * @brief Use the default constructor for the serial client
   *
   * @note We will use the @ref ConnectClient method to handle most of our setup
   *
   */
  explicit SerialClient(const rclcpp::Logger & logger);

  /**
   * @brief
   *
   * @param device
   * @param timeout_ms
   */
  void ConnectClient(const std::string & device, int timeout_ms = 500);

  /**
   * @brief Shutdown the serial client.
   *
   * @note The main intention for this method is to shutdown the RX thread. While we could
   * accomplish this in the destructor, that might not be safe if the thread experiences unknown
   * behavior on shutdown. In the future, if this ultimately demonstrates itself to be safe, then
   * this method will be eliminated in favor of shutting down in the destructor for usability
   * purposes.
   *
   */
  void DisconnectClient();

  /**
   * @brief Send a packet over the serial connection.
   *
   * @param packet message to send to the Reach Alpha manipulator; the data
   * should not yet be encoded.
   */
  void Send(const Packet & packet) const;

  /**
   * @brief Register a new callback function to a specified packet type.
   *
   * @param packet_type type of packet that the callback should be signaled on
   * @param callback function that should be executed when a message of a given
   * type is received
   */
  void RegisterCallback(PacketId packet_type, const std::function<void(Packet)> & callback);

  /**
   * @brief Indicates whether or not the arm connection is currently active.
   *
   * @note To be considered 'active' there must be an open serial connection and the
   * client must be actively receiving heartbeat packets from the Reach Alpha manipulator.
   *
   * @return true
   * @return false
   */
  bool active() const;  // NOLINT

private:
  /**
   * @brief Defines the current state of the serial port.
   *
   */
  enum class PortState
  {
    kOpen,   // The serial port is currently open
    kClosed  // The serial port is currently closed
  };

  /**
   * @brief Method used to poll the serial line for incoming data.
   *
   * @note This method is executed by the RX thread. Furthermore, this method is
   * a blocking method that runs indefinitely. The main loop is terminated by
   * the client status flag.
   *
   */
  void Read();

  // Map used to store the callback functions for messages.
  // Keys should be the ID of a message and the values are the callback
  // functions to execute when a message the respective packet ID is received.
  std::unordered_map<PacketId, std::vector<std::function<void(Packet)>>> callbacks_;

  // Serial port file descriptor
  int fd_;

  std::atomic<bool> running_{false};  // This flag is used to stop the RX main loop
  PortState port_status_ = PortState::kClosed;

  // Thread responsible for receiving incoming data and executing the respective
  // callbacks
  std::thread rx_worker_;

  // We use the built-in ROS logger here for the sake of logging consistency
  // This logger also uses spdlog which was what I would have used anyway
  rclcpp::Logger logger_;
};

}  // namespace alpha_driver
