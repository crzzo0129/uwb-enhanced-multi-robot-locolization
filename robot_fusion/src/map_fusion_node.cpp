#include "robot_fusion/map_fusion_node.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>

namespace robot_fusion
{

MapFusionNode::MapFusionNode(const rclcpp::NodeOptions & options)
: Node("map_fusion_node", options)
{
  // 声明参数
  world_frame_ = this->declare_parameter("world_frame", "world");
  merged_topic_ = this->declare_parameter("merged_map_topic", "/merged_map");
  resolution_ = this->declare_parameter("map_resolution", 0.05);
  update_rate_ = this->declare_parameter("update_rate", 2.0);

  RCLCPP_INFO(this->get_logger(), "Map Fusion Node 启动...");
  RCLCPP_INFO(this->get_logger(), "World frame: %s, Resolution: %.3f, Update rate: %.1f Hz", 
              world_frame_.c_str(), resolution_, update_rate_);

  // 订阅三个地图（使用 lambda 传递 robot_name）
  sub_rb1_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/rb1/map", 10, 
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { mapCallback(msg, "rb1"); });
    
  sub_rb2_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/rb2/map", 10, 
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { mapCallback(msg, "rb2"); });
    
  sub_rb3_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/rb3/map", 10, 
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { mapCallback(msg, "rb3"); });

  // 发布合并地图
  merged_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(merged_topic_, 10);

  // 创建定时器（毫秒周期）
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
  timer_ = this->create_wall_timer(timer_period, std::bind(&MapFusionNode::mergeMaps, this));
}

void MapFusionNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, 
                                const std::string& robot_name)
{
  maps_[robot_name] = *msg;
  RCLCPP_DEBUG(this->get_logger(), "收到 %s 地图: %dx%d", 
               robot_name.c_str(), msg->info.width, msg->info.height);
}

MapFusionNode::MapBounds MapFusionNode::calculateBounds(const nav_msgs::msg::OccupancyGrid& map)
{
  MapBounds bounds;
  bounds.min_x = map.info.origin.position.x;
  bounds.min_y = map.info.origin.position.y;
  bounds.max_x = bounds.min_x + map.info.width * map.info.resolution;
  bounds.max_y = bounds.min_y + map.info.height * map.info.resolution;
  return bounds;
}

void MapFusionNode::mergeMaps()
{
  if (maps_.empty()) {
    return;
  }

  // 计算所有地图的边界框（世界坐标系）
  double global_min_x = std::numeric_limits<double>::max();
  double global_min_y = std::numeric_limits<double>::max();
  double global_max_x = std::numeric_limits<double>::lowest();
  double global_max_y = std::numeric_limits<double>::lowest();

  for (const auto& [name, map] : maps_) {
    auto bounds = calculateBounds(map);
    global_min_x = std::min(global_min_x, bounds.min_x);
    global_min_y = std::min(global_min_y, bounds.min_y);
    global_max_x = std::max(global_max_x, bounds.max_x);
    global_max_y = std::max(global_max_y, bounds.max_y);
  }

  // 计算合并后地图尺寸（像素）
  int new_width = static_cast<int>((global_max_x - global_min_x) / resolution_);
  int new_height = static_cast<int>((global_max_y - global_min_y) / resolution_);

  if (new_width <= 0 || new_height <= 0) {
    RCLCPP_WARN(this->get_logger(), "地图尺寸计算错误: %dx%d", new_width, new_height);
    return;
  }

  // 创建空白地图数据（-1 = 未知，使用 int8_t）
  std::vector<int8_t> merged_data(new_width * new_height, -1);

  // 融合每个地图
  for (const auto& [name, map] : maps_) {
    int map_width = static_cast<int>(map.info.width);
    int map_height = static_cast<int>(map.info.height);
    
    // 计算该地图在合并地图中的起始像素位置
    int offset_x = static_cast<int>((map.info.origin.position.x - global_min_x) / resolution_);
    int offset_y = static_cast<int>((map.info.origin.position.y - global_min_y) / resolution_);

    // 遍历地图每个像素
    for (int y = 0; y < map_height; ++y) {
      for (int x = 0; x < map_width; ++x) {
        // 源地图索引
        int src_idx = y * map_width + x;
        int8_t value = map.data[src_idx];

        // 目标位置
        int dst_x = offset_x + x;
        int dst_y = offset_y + y;
        
        // 边界检查
        if (dst_x < 0 || dst_x >= new_width || dst_y < 0 || dst_y >= new_height) {
          continue;
        }
        
        int dst_idx = dst_y * new_width + dst_x;
        int8_t& current = merged_data[dst_idx];

        // 融合策略：保守策略（占用优先）
        // -1=未知, 0=空闲, 100=占用
        if (value == 100) {  // 如果源地图显示占用
          current = 100;
        } else if (value == 0 && current != 100) {  // 空闲且当前未被占用
          current = 0;
        }
        // -1 不覆盖已有值
      }
    }
  }

  // 构造并发布消息
  nav_msgs::msg::OccupancyGrid merged_msg;
  merged_msg.header.stamp = this->now();
  merged_msg.header.frame_id = world_frame_;
  
  merged_msg.info.resolution = static_cast<float>(resolution_);
  merged_msg.info.width = static_cast<uint32_t>(new_width);
  merged_msg.info.height = static_cast<uint32_t>(new_height);
  merged_msg.info.origin.position.x = global_min_x;
  merged_msg.info.origin.position.y = global_min_y;
  merged_msg.info.origin.position.z = 0.0;
  merged_msg.info.origin.orientation.w = 1.0;
  
  merged_msg.data = std::move(merged_data);
  
  merged_pub_->publish(merged_msg);
  
  RCLCPP_DEBUG(this->get_logger(), "发布合并地图: %dx%d, 原点: (%.2f, %.2f)", 
               new_width, new_height, global_min_x, global_min_y);
}

} // namespace robot_fusion

