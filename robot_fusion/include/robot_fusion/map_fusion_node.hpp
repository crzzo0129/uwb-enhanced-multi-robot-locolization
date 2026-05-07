#ifndef MAP_FUSION_NODE_HPP_
#define MAP_FUSION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <string>
#include <vector>
#include <cstdint>

namespace robot_fusion
{

class MapFusionNode : public rclcpp::Node
{
public:
  explicit MapFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string& robot_name);
  void mergeMaps();
  
  // 计算单张地图在世界坐标系下的边界
  struct MapBounds {
    double min_x, min_y, max_x, max_y;
  };
  MapBounds calculateBounds(const nav_msgs::msg::OccupancyGrid& map);

  // 缓存最新地图
  std::map<std::string, nav_msgs::msg::OccupancyGrid> maps_;
  
  // 订阅者
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_rb1_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_rb2_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_rb3_;
  
  // 发布者
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 参数
  std::string world_frame_;
  std::string merged_topic_;
  double resolution_;
  double update_rate_;
};

} // namespace robot_fusion

#endif // MAP_FUSION_NODE_HPP_