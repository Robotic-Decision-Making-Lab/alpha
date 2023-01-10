#pragma once

#include <stdint.h>

#include <vector>

#include "alpha_driver/device_id.hpp"
#include "alpha_driver/packet_id.hpp"

namespace alpha_driver
{

class Packet
{
public:
  /**
   * @brief
   *
   * @param packet_id
   * @param device_id
   */
  Packet(PacketId packet_id, DeviceId device_id);

  /**
   * @brief
   *
   * @param packet_id
   * @param device_id
   * @param data
   */
  Packet(PacketId packet_id, DeviceId device_id, std::vector<unsigned char> data);

  /**
   * @brief
   *
   * @param data
   * @return Packet
   */
  // static Packet Decode(const std::vector<unsigned char> & data);

  /**
   * @brief
   *
   * @return std::vector<unsigned char>
   */
  auto Encode() const -> std::vector<unsigned char>;

private:
  PacketId packet_id_;
  DeviceId device_id_;
  std::vector<unsigned char> data_;
};

}  // namespace alpha_driver
