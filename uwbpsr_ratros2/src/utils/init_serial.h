#ifndef INITSERIAL_H
#define INITSERIAL_H
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>

void initSerial(serial::Serial *serial, rclcpp::Node::SharedPtr node);

#endif // INITSERIAL_H
