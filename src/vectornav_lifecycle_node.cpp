#include <rclcpp/rclcpp.hpp>
#include <vectornav/vectornav_lifecycle.hpp>

int main(int argc, char ** argv)
{
  try {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<vectornav::VectornavLifecycleNode>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("vectornav"), "Exception: %s", e.what());
    return 1;
  }
  return 0;
}