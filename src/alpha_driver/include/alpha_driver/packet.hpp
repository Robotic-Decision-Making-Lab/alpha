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

#include <vector>

#include "alpha_driver/device_id.hpp"
#include "alpha_driver/packet_id.hpp"

namespace alpha_driver
{

class Packet
{
public:
  /**
   * @brief Use the default copy constructor for Packets
   */
  Packet(const Packet &) = default;

  /**
   * @brief Create a new Packet.
   *
   * @remark Each packet has been designed to be immutable, and, therefore, requires the packet
   * data to be provided at construction.
   *
   * @param packet_id type of packet
   * @param device_id ID of the device that the data represents
   * @param data packet data
   */
  Packet(PacketId packet_id, DeviceId device_id, std::vector<unsigned char> data);

  /**
   * @brief Encode the packet's data using the BPL packet structure specification.
   *
   * @return std::vector<unsigned char>
   */
  std::vector<unsigned char> Encode() const;

  /**
   * @brief Decode a packet that has been encoded using the BPL communication
   * specification.
   *
   * @param data
   * @return Packet
   */
  static Packet Decode(const std::vector<unsigned char> & data);

  /**
   * @brief Get the unique packet identifier for a given message.
   *
   * @return PacketId
   */
  PacketId packet_id() const;

  /**
   * @brief Get the unique device ID that the data represents.
   *
   * @return DeviceId
   */
  DeviceId device_id() const;

  /**
   * @brief Get the packet data.
   *
   * @return std::vector<unsigned char>
   */
  std::vector<unsigned char> data() const;

private:
  PacketId packet_id_;
  DeviceId device_id_;
  std::vector<unsigned char> data_;
};

}  // namespace alpha_driver
