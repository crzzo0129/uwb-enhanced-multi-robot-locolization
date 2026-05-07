#include "robot_fusion/map_fusion_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robot_fusion::MapFusionNode>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}