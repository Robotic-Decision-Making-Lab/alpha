#include <gmock/gmock.h>

#include "alpha_driver/cobs.hpp"

namespace alpha_driver_test
{

TEST(CobsTest, EncodesData)
{
  std::vector<unsigned char> data = {0x23, 0x00, 0xD4, 0x81, 0x00, 0xFA};
  std::vector<unsigned char> encoded_data = {0x02, 0x23, 0x03, 0xD4, 0x81, 0x02, 0xFA, 0x00};

  ASSERT_THAT(alpha_driver::CobsEncode(data), ::testing::ElementsAreArray(encoded_data));
};

TEST(CobsTest, DecodesData)
{
  std::vector<unsigned char> encoded_data = {0x02, 0x23, 0x03, 0xD4, 0x81, 0x02, 0xFA, 0x00};
  std::vector<unsigned char> decoded_data = {0x23, 0x00, 0xD4, 0x81, 0x00, 0xFA};

  ASSERT_THAT(alpha_driver::CobsDecode(encoded_data), ::testing::ElementsAreArray(decoded_data));
}

}  // namespace alpha_driver_test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  return result;
}
