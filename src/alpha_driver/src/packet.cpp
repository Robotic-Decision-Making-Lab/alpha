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

#include "alpha_driver/packet.hpp"

#include <cstdint>
#include <utility>
#include <vector>

#include "alpha_driver/cobs.hpp"
#include "alpha_driver/crc.hpp"

namespace alpha_driver
{

Packet::Packet(PacketId packet_id, DeviceId device_id)
: packet_id_(packet_id),
  device_id_(device_id)
{
}

Packet::Packet(PacketId packet_id, DeviceId device_id, std::vector<unsigned char> data)
: packet_id_(packet_id),
  device_id_(device_id),
  data_(std::move(data))
{
}

// unsigned char Packet::Decode(const std::vector<unsigned char> & data){};

auto Packet::Encode() const -> std::vector<unsigned char>
{
  std::vector<unsigned char> data(data_);

  // Add the packet ID and the device ID
  // Note that we need to type cast to the underlying type because enum classes
  // don't implicitly cast to ints (which is a good thing)
  data.push_back(static_cast<std::underlying_type<PacketId>::type>(packet_id_));
  data.push_back(static_cast<std::underlying_type<DeviceId>::type>(device_id_));

  // Length is the current buffer size plus two (length and CRC)
  data.push_back(data.size() + 2);

  // Calculate the CRC from the data and add it to the buffer
  data.push_back(CalculateBplCrc8(data_));

  // Encode the data using COBS encoding
  std::vector<unsigned char> encoded_data = CobsEncode(data);

  return encoded_data;
}

}  // namespace alpha_driver
