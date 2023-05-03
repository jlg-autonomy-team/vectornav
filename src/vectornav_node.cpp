#include <rclcpp/rclcpp.hpp>
#include <vectornav/vectornav.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vectornav::Vectornav>();
  rclcpp::spin(node);
  return 0;
}