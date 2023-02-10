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

#include <gmock/gmock.h>

#include "alpha_driver/crc.hpp"
#include "alpha_driver/packet.hpp"
#include "alpha_driver/packet_id.hpp"
#include "alpha_driver/serial_client.hpp"

namespace test_alpha_driver
{

TEST(PacketTest, TestPacketEncode)
{
  const std::vector<unsigned char> data = {
    static_cast<unsigned char>(alpha_driver::PacketId::kPosition)};

  // Create an encoded test packet using the BPL structure based off of the test
  // data
  const std::vector<unsigned char> expected_encoding = {0x06, 0x03, 0x60, 0x01, 0x05, 0x52, 0x00};

  // Construct a new packet using the data and the expected IDs
  auto packet = alpha_driver::Packet(
    alpha_driver::PacketId::kRequest, alpha_driver::DeviceId::kLinearJaws, data);

  ASSERT_THAT(packet.encode(), ::testing::ElementsAreArray(expected_encoding));
}

TEST(PacketTest, TestPacketDecode)
{
  const std::vector<unsigned char> encoded_data = {0x09, 0x01, 0x02, 0x03, 0x04,
                                                   0x01, 0xFF, 0x08, 0x5D, 0x00};

  const std::vector<unsigned char> decoded_data = {0x01, 0x02, 0x03, 0x04};

  const alpha_driver::Packet packet = alpha_driver::Packet::decode(encoded_data);

  ASSERT_THAT(packet.data(), ::testing::ElementsAreArray(decoded_data));
}

TEST(PacketTest, TestInvalidDecoding)
{
  const std::vector<unsigned char> decoded_data = {0x01, 0x02, 0x03, 0x04};

  // Cannot decoded data that has already been decoded
  ASSERT_THROW(alpha_driver::Packet::decode(decoded_data), std::runtime_error);
}

TEST(PacketTest, TestInvalidPacketConstruction)
{
  const std::vector<unsigned char> empty_data = {};

  ASSERT_THROW(
    alpha_driver::Packet(
      alpha_driver::PacketId::kVelocity, alpha_driver::DeviceId::kAllJoints, empty_data),
    std::invalid_argument);
}

}  // namespace test_alpha_driver

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
