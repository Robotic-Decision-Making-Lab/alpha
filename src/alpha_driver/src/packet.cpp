#include "alpha_driver/packet.hpp"

#include <stdint.h>

#include <vector>

#include "alpha_driver/crc.hpp"

namespace alpha_driver
{

Packet::Packet(PacketId packet_id, DeviceId data_id) : packet_id_(packet_id), device_id_(data_id){};

Packet::Packet(PacketId packet_id, DeviceId data_id, std::vector<unsigned char> data)
: packet_id_(packet_id), device_id_(data_id), data_(data){};

// unsigned char Packet::Decode(const std::vector<unsigned char> & data){};

std::vector<unsigned char> Packet::Encode() const
{
  std::vector<unsigned char> data(data_);

  // Add the packet ID and the device ID
  // Note that we need to type cast to the underlying type because enum classes
  // don't implicitly cast to ints (which is a good thing)
  data.push_back(static_cast<std::underlying_type<PacketId>::type>(packet_id_));
  data.push_back(static_cast<std::underlying_type<DeviceId>::type>(device_id_));

  // Length is the current buffer plus two (length and CRC)
  data.push_back(data.size() + 2);

  // TODO: calculate the CRC and add it to the buffer
  // TODO: encode the COBS and add it to the buffer
  // TODO: add the terminator (0x00) to the buffer
};

}  // namespace alpha_driver
