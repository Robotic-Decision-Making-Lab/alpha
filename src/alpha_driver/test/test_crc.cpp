#include <gtest/gtest.h>

#include "alpha_driver/crc.hpp"

namespace alpha_driver_test
{

TEST(CrcTest, CalculatesCorrectBplCrc)
{
  std::vector<unsigned char> message = {0xAA, 0xD8, 0x92, 0x84, 0x75};
  const unsigned char expected_crc = 0xD7;

  EXPECT_EQ(alpha_driver::CalculateBplCrc8(message, message.size()), expected_crc);
}

}  // namespace alpha_driver_test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  return result;
}
