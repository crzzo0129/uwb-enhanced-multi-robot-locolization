#include "init_serial.h"

#include "rclcpp/rclcpp.hpp"
#include <string>

void initSerial(serial::Serial *serial, rclcpp::Node::SharedPtr node) {
  try {
    // 声明参数并设置默认值
    node->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    node->declare_parameter<int>("baud_rate", 921600);

    // 获取参数
    std::string port_name;
    int baud_rate;
    node->get_parameter("port_name", port_name);
    node->get_parameter("baud_rate", baud_rate);

    serial->setPort(port_name);
    serial->setBaudrate(static_cast<uint32_t>(baud_rate));

    RCLCPP_INFO(node->get_logger(), "Try to open serial port with %s, %d", port_name.c_str(), baud_rate);

    auto timeout = serial::Timeout::simpleTimeout(10);
    // without setTimeout, serial can not write any data
    serial->setTimeout(timeout);
    serial->open();

    if (serial->isOpen()) {
      RCLCPP_INFO(node->get_logger(), "Serial port opened successfully, waiting for data.");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to open serial port, please check and retry.");
      exit(EXIT_FAILURE);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Unhandled Exception: %s", e.what());
    exit(EXIT_FAILURE);
  }
}
