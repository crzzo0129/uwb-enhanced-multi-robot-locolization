#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <unordered_map>

#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"

class NProtocolExtracter;

namespace linktrack {

class Init {
public:
  Init(rclcpp::Node::SharedPtr node,
       NProtocolExtracter *protocol_extraction,
       serial::Serial *serial);

private:
  void initDataTransmission();
  void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
  void initTagFrame0(NProtocolExtracter *protocol_extraction);
  void initNodeFrame0(NProtocolExtracter *protocol_extraction);
  void initNodeFrame1(NProtocolExtracter *protocol_extraction);
  void initNodeFrame2(NProtocolExtracter *protocol_extraction);
  void initNodeFrame3(NProtocolExtracter *protocol_extraction);
  void initNodeFrame4(NProtocolExtracter *protocol_extraction);
  void initNodeFrame5(NProtocolExtracter *protocol_extraction);
  void initNodeFrame6(NProtocolExtracter *protocol_extraction);

  // 协议对象和 Publisher 的映射
  std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;

  // 节点指针
  rclcpp::Node::SharedPtr node_;

  // 数据传输订阅
  rclcpp::SubscriptionBase::SharedPtr dt_sub_;
};

} // namespace linktrack

#endif // LINKTRACKINIT_H
