#include "alpha_driver/cobs.hpp"

#include <vector>
#include <stdint.h>

namespace alpha_driver
{

std::vector<unsigned char> CobsEncode(const std::vector<unsigned char> & data)
{
  // // Initialize the encoded data with 0x00 prepended
  // // this will be overwritten once the count to the next 0x00 is detemined
  std::vector<unsigned char> encoded_data = {0x00};

  int block_start = 0;
  int current_block_size = 0;

  for (auto it = data.begin(); it != data.end(); ++it) {
    if (*it == 0x00) {
      // Save the total number of elements before the next 0x00
      encoded_data[block_start] = (uint8_t)(current_block_size + 1);

      // Add a placeholder
      encoded_data.push_back(0x00);

      // Reset the counters
      block_start = encoded_data.size() - 1;
      current_block_size = 0;
    } else {
      // Copy over the data
      encoded_data.push_back(*it);
      current_block_size++;

      // Handle the case where the block size is 254 or greater
      if (current_block_size >= 254) {
        encoded_data[block_start] = (uint8_t)(current_block_size + 1);

        // Add placeholder
        encoded_data.push_back(0x00);

        // Reset counters
        block_start = encoded_data.size() - 1;
        current_block_size = 0;
      }
    }
  }

  encoded_data[block_start] = (uint8_t)(current_block_size + 1);
  encoded_data.push_back(0x00);

  return encoded_data;
}

} // namespace alpha_driver
