#include "stereo_depth_cpp/stereobm_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_depth_cpp::StereoBMNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
