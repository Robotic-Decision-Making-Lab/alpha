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

#include <gmock/gmock.h>

#include "alpha_driver/cobs.hpp"

namespace alpha_driver_test
{

TEST(CobsTest, EncodesData)
{
  const std::vector<unsigned char> data = {0x23, 0x00, 0xD4, 0x81, 0x00, 0xFA};
  const std::vector<unsigned char> encoded_data = {0x02, 0x23, 0x03, 0xD4, 0x81, 0x02, 0xFA, 0x00};

  ASSERT_THAT(alpha_driver::cobsEncode(data), ::testing::ElementsAreArray(encoded_data));
}

TEST(CobsTest, DecodesData)
{
  const std::vector<unsigned char> encoded_data = {0x02, 0x2, 0x03, 0xD4, 0x81, 0x02, 0xFA, 0x00};
  const std::vector<unsigned char> decoded_data = {0x23, 0x00, 0xD4, 0x81, 0x00, 0xFA};

  ASSERT_THAT(alpha_driver::cobsDecode(encoded_data), ::testing::ElementsAreArray(decoded_data));
}

}  // namespace alpha_driver_test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
