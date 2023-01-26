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

#include "alpha_driver/crc.hpp"

namespace alpha_driver
{

unsigned char Reflect(std::uint64_t data, int size)
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

unsigned char CalculateCrc8(
  const std::vector<unsigned char> & data, unsigned char initial_value,
  unsigned char final_xor_value, bool input_reflected, bool result_reflected,
  const std::array<unsigned char, 256> & lookup_table)
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

unsigned char CalculateBplCrc8(const std::vector<unsigned char> & data)
{
  return CalculateCrc8(
    data, kInitialValue, kFinalXorValue, kInputReflected, kResultReflected, kCrc8LookupTable);
}

}  // namespace alpha_driver
