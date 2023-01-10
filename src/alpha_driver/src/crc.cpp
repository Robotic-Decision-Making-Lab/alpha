#include "alpha_driver/crc.hpp"

#include <stdint.h>

namespace alpha_driver
{

unsigned char Reflect(unsigned long data, int size)
{
  unsigned long reflection = 0x00000000;
  unsigned char bit;

  for (bit = 0; bit < size; ++bit) {
    if (data & 0x01) {
      reflection |= (1 << ((size - 1) - bit));
    }

    data = (data >> 1);
  }

  return (unsigned char)reflection;
}

unsigned char CalculateCrc8(
  const std::vector<unsigned char> & data, unsigned char initial_value,
  unsigned char final_xor_value, bool input_reflected, bool result_reflected,
  const std::array<unsigned char, 256> & lookup_table)
{
  unsigned char crc = initial_value;
  unsigned char value;

  const int kWidth = (8 * sizeof(crc));

  for (std::vector<int>::size_type i = 0; i < data.size(); ++i) {
    // Reflect the data
    if (input_reflected) {
      value = Reflect(data[i], 8);
    } else {
      value = data[i];
    }

    value ^= crc >> (kWidth - 8);
    crc = lookup_table[value] ^ (crc << 8);
  }

  // Reflect the result
  if (result_reflected) {
    crc = Reflect(crc, kWidth);
  }

  return crc ^ final_xor_value;
}

unsigned char CalculateBplCrc8(const std::vector<unsigned char> & data)
{
  return CalculateCrc8(
    data, kInitialValue, kFinalXorValue, kInputReflected, kResultReflected, kCrc8LookupTable);
}

}  // namespace alpha_driver
