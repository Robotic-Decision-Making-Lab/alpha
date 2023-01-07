#pragma once

#include <vector>

namespace alpha_driver
{

/**
 * @brief
 *
 * @param in
 * @return std::vector<unsigned char>
 */
std::vector<unsigned char> CobsEncode(const std::vector<unsigned char> & in);

/**
 * @brief
 *
 * @param in
 * @return std::vector<unsigned char>
 */
std::vector<unsigned char> CobsDecode(const std::vector<unsigned char> & in);

}  // namespace alpha_driver
