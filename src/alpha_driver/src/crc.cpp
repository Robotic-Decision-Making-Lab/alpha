#include "alpha_driver/crc.hpp"

#include <cstdint>

namespace alpha_driver
{

auto Reflect(std::uint64_t data, int size) -> unsigned char
{
  std::uint64_t reflection = 0x00000000;

  for (int bit = 0; bit < size; ++bit) {
    if (data & 0x01) {
      reflection |= (1 << ((size - 1) - bit));
    }

    data = (data >> 1);
  }

  return static_cast<unsigned char>(reflection);
}

auto CalculateCrc8(
  const std::vector<unsigned char> & data, unsigned char initial_value,
  unsigned char final_xor_value, bool input_reflected, bool result_reflected,
  const std::array<unsigned char, 256> & lookup_table) -> unsigned char
{
  unsigned char crc = initial_value;
  unsigned char value;

  const int width = (8 * sizeof(crc));

  for (const unsigned char byte : data) {
    // Reflect the data
    if (input_reflected) {
      value = Reflect(byte, 8);
    } else {
      value = byte;
    }

    value ^= crc >> (width - 8);
    crc = lookup_table[value] ^ (crc << 8);
  }

  // Reflect the result
  if (result_reflected) {
    crc = Reflect(crc, width);
  }

  return crc ^ final_xor_value;
}

auto CalculateBplCrc8(const std::vector<unsigned char> & data) -> unsigned char
{
  return CalculateCrc8(
    data, kInitialValue, kFinalXorValue, kInputReflected, kResultReflected, kCrc8LookupTable);
}

}  // namespace alpha_driver
