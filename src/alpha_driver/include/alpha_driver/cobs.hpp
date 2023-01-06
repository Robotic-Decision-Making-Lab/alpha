#pragma once

#include <vector>

namespace alpha_driver {

/**
 * @brief
 *
 * @param in
 * @return std::vector<unsigned char>
 */
std::vector<unsigned char> cobsEncode(const std::vector<unsigned char>& in);

/**
 * @brief
 *
 * @param in
 * @return std::vector<unsigned char>
 */
std::vector<unsigned char> cobsDecode(const std::vector<unsigned char>& in);

}  // namespace alpha_driver
