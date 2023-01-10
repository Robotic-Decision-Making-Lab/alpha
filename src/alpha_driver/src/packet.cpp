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
