#include "init.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nutils.h"
#include "protocols.h"

#include "uwbpsr_ratros2/msg/linktrack_anchorframe0.hpp"
#include "uwbpsr_ratros2/msg/linktrack_tagframe0.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe0.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe1.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe2.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe3.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe4.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe5.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe6.hpp"

#define ARRAY_ASSIGN(DEST, SRC) \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) { \
    DEST[_CNT] = SRC[_CNT]; \
  }

using std::placeholders::_1;

namespace linktrack {

using namespace uwbpsr_ratros2::msg;

LinktrackAnchorframe0 g_msg_anchorframe0;
LinktrackTagframe0 g_msg_tagframe0;
LinktrackNodeframe0 g_msg_nodeframe0;
LinktrackNodeframe1 g_msg_nodeframe1;
LinktrackNodeframe2 g_msg_nodeframe2;
LinktrackNodeframe3 g_msg_nodeframe3;
LinktrackNodeframe4 g_msg_nodeframe4;
LinktrackNodeframe5 g_msg_nodeframe5;
LinktrackNodeframe6 g_msg_nodeframe6;

serial::Serial *serial_;

Init::Init(rclcpp::Node::SharedPtr node,
           NProtocolExtracter *protocol_extraction,
           serial::Serial *serial)
  : node_(node)
{
  serial_ = serial;
  initDataTransmission();
  initAnchorFrame0(protocol_extraction);
  initTagFrame0(protocol_extraction);
  initNodeFrame0(protocol_extraction);
  initNodeFrame1(protocol_extraction);
  initNodeFrame2(protocol_extraction);
  initNodeFrame3(protocol_extraction);
  initNodeFrame4(protocol_extraction);
  initNodeFrame5(protocol_extraction);
  initNodeFrame6(protocol_extraction);
}

void Init::initDataTransmission()
{
  dt_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "nlink_linktrack_data_transmission", 10,
    [](const std_msgs::msg::String::SharedPtr msg) {
      if (serial_)
        serial_->write(msg->data);
    });
}

#define CREATE_AND_PUBLISH(PROTOCOL, MSGTYPE, TOPIC, MSGOBJ, DATAVAR, ...) \
  auto protocol = new PROTOCOL; \
  protocol_extraction->AddProtocol(protocol); \
  protocol->SetHandleDataCallback([=]() mutable { \
    if (!publishers_.count(protocol)) { \
      publishers_[protocol] = node_->create_publisher<MSGTYPE>(TOPIC, 10); \
      RCLCPP_INFO(node_->get_logger(), "Advertised topic: %s", TOPIC); \
    } \
    auto &msg = MSGOBJ; \
    auto &data = DATAVAR; \
    __VA_ARGS__ \
    std::static_pointer_cast<rclcpp::Publisher<MSGTYPE>>(publishers_[protocol])->publish(msg); \
  });

void Init::initAnchorFrame0(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolAnchorFrame0, LinktrackAnchorframe0, "nlink_linktrack_anchorframe0", g_msg_anchorframe0, nlt_anchorframe0_.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.voltage = data.voltage;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;
    msg.nodes.clear();
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto node = data.nodes[i];
      decltype(msg.nodes)::value_type msg_node;
      msg_node.role = node->role;
      msg_node.id = node->id;
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d);
      ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr);
      msg.nodes.push_back(msg_node);
    }
  )
}

void Init::initTagFrame0(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolTagFrame0, LinktrackTagframe0, "nlink_linktrack_tagframe0", g_msg_tagframe0, g_nlt_tagframe0.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;
    msg.voltage = data.voltage;
    ARRAY_ASSIGN(msg.pos_3d, data.pos_3d);
    ARRAY_ASSIGN(msg.eop_3d, data.eop_3d);
    ARRAY_ASSIGN(msg.vel_3d, data.vel_3d);
    ARRAY_ASSIGN(msg.dis_arr, data.dis_arr);
    ARRAY_ASSIGN(msg.imu_gyro_3d, data.imu_gyro_3d);
    ARRAY_ASSIGN(msg.imu_acc_3d, data.imu_acc_3d);
    ARRAY_ASSIGN(msg.angle_3d, data.angle_3d);
    ARRAY_ASSIGN(msg.quaternion, data.quaternion);
  )
}

void Init::initNodeFrame0(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolNodeFrame0, LinktrackNodeframe0, "nlink_linktrack_nodeframe0", g_msg_nodeframe0, g_nlt_nodeframe0.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg.nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.data.resize(node->data_length);
      memcpy(msg_node.data.data(), node->data, node->data_length);
    }
  )
}

void Init::initNodeFrame1(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolNodeFrame1, LinktrackNodeframe1, "nlink_linktrack_nodeframe1", g_msg_nodeframe1, g_nlt_nodeframe1.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;
    msg.voltage = data.voltage;
    msg.nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg.nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d);
    }
  )
}

void Init::initNodeFrame2(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolNodeFrame2, LinktrackNodeframe2, "uwb", g_msg_nodeframe2, g_nlt_nodeframe2.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;
    msg.voltage = data.voltage;
    ARRAY_ASSIGN(msg.pos_3d, data.pos_3d);
    ARRAY_ASSIGN(msg.eop_3d, data.eop_3d);
    ARRAY_ASSIGN(msg.vel_3d, data.vel_3d);
    ARRAY_ASSIGN(msg.imu_gyro_3d, data.imu_gyro_3d);
    ARRAY_ASSIGN(msg.imu_acc_3d, data.imu_acc_3d);
    ARRAY_ASSIGN(msg.angle_3d, data.angle_3d);
    ARRAY_ASSIGN(msg.quaternion, data.quaternion);
    msg.nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg.nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }
  )
}

void Init::initNodeFrame3(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolNodeFrame3, LinktrackNodeframe3, "nlink_linktrack_nodeframe3", g_msg_nodeframe3, g_nlt_nodeframe3.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;
    msg.voltage = data.voltage;
    msg.nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg.nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }
  )
}

void Init::initNodeFrame4(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolNodeFrame4, LinktrackNodeframe4, "nlink_linktrack_nodeframe4", g_msg_nodeframe4, g_nlt_nodeframe4.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;
    msg.voltage = data.voltage;
    msg.tags.resize(data.tag_count);
    for (int i = 0; i < data.tag_count; ++i) {
      auto &msg_tag = msg.tags[i];
      auto tag = data.tags[i];
      msg_tag.id = tag->id;
      msg_tag.voltage = tag->voltage;
      msg_tag.anchors.resize(tag->anchor_count);
      for (int j = 0; j < tag->anchor_count; ++j) {
        auto &msg_anchor = msg_tag.anchors[j];
        auto anchor = tag->anchors[j];
        msg_anchor.id = anchor->id;
        msg_anchor.dis = anchor->dis;
      }
    }
  )
}

void Init::initNodeFrame5(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolNodeFrame5, LinktrackNodeframe5, "nlink_linktrack_nodeframe5", g_msg_nodeframe5, g_nlt_nodeframe5.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.local_time = data.local_time;
    msg.system_time = data.system_time;
    msg.voltage = data.voltage;
    msg.nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg.nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }
  )
}

void Init::initNodeFrame6(NProtocolExtracter *protocol_extraction) {
  CREATE_AND_PUBLISH(NLT_ProtocolNodeFrame6, LinktrackNodeframe6, "nlink_linktrack_nodeframe6", g_msg_nodeframe6, g_nlt_nodeframe6.result,
    msg.role = data.role;
    msg.id = data.id;
    msg.nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg.nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.data.resize(node->data_length);
      memcpy(msg_node.data.data(), node->data, node->data_length);
    }
  )
}

} // namespace linktrack
