#include "elas_ros2_cpp/elas_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<elas_ros2_cpp::ELASNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


