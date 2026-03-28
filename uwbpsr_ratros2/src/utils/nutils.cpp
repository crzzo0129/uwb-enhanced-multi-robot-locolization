#include "nutils.h"
#include "rclcpp/rclcpp.hpp"

void TopicAdvertisedTip(const char *topic) {
  // 定义静态 logger，名字你可以自定义
  static rclcpp::Logger logger = rclcpp::get_logger("uwbpsr_ratros2.nutils");

  RCLCPP_INFO(logger, "%s has been advertised, use 'ros2 topic echo /%s' to view the data", topic, topic);
}
