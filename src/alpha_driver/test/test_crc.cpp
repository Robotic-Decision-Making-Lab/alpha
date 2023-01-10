#include <gtest/gtest.h>

#include "alpha_driver/crc.hpp"

namespace alpha_driver_test
{

TEST(CrcTest, CalculatesBplCrc)
{
  std::vector<unsigned char> message = {0xFF, 0x12, 0xAD, 0x23, 0x56};
  const unsigned char expected_crc = 0xF3;

  EXPECT_EQ(alpha_driver::CalculateBplCrc8(message), expected_crc);
}

}  // namespace alpha_driver_test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  return result;
}
