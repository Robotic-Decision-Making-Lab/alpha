#include "alpha_driver/serial_client.hpp"

#include "alpha_driver/packet.hpp"

namespace alpha_driver
{

SerialClient::SerialClient() : Node("SerialClient"){};

}  // namespace alpha_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<alpha_driver::SerialClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
