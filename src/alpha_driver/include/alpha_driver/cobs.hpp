#pragma once

#include <vector>

namespace alpha_driver
{

/**
 * @brief Encode the data using the consistent overhead bytes stuffing (COBS) algorithm. This
 * implementation has been inspired by the following source:
 * https://github.com/gbmhunter/SerialFiller/blob/d678acbf6d29de7042d48c6be8ecef556bb6d857/src/CobsTranscoder.cpp#L19
 *
 * @param data
 * @return std::vector<unsigned char>
 */
std::vector<unsigned char> CobsEncode(const std::vector<unsigned char> & data);

/**
 * @brief Decode a data packet that has been encoded using the COBS algorithm. This implementation
 * has been inspired by the following source:
 * https://github.com/gbmhunter/SerialFiller/blob/d678acbf6d29de7042d48c6be8ecef556bb6d857/src/CobsTranscoder.cpp#L74
 *
 * @param data
 * @return std::vector<unsigned char>
 */
std::vector<unsigned char> CobsDecode(const std::vector<unsigned char> & data);

}  // namespace alpha_driver
