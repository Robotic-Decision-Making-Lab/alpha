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
auto CobsEncode(const std::vector<unsigned char> & data) -> std::vector<unsigned char>;

/**
 * @brief Decode a data packet that has been encoded using the COBS algorithm. This implementation
 * has been inspired by the following source:
 * https://github.com/gbmhunter/SerialFiller/blob/d678acbf6d29de7042d48c6be8ecef556bb6d857/src/CobsTranscoder.cpp#L74
 *
 * @param data
 * @return std::vector<unsigned char>
 */
auto CobsDecode(const std::vector<unsigned char> & data) -> std::vector<unsigned char>;

}  // namespace alpha_driver
