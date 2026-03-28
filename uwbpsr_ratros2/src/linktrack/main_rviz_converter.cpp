#include <memory>
#include <map>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "uwbpsr_ratros2/msg/linktrack_anchorframe0.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe1.hpp"
#include "uwbpsr_ratros2/msg/linktrack_nodeframe2.hpp"
#include "uwbpsr_ratros2/msg/linktrack_tagframe0.hpp"

namespace {

using std::placeholders::_1;

std::string frame_id;

struct PosePair {
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
  geometry_msgs::msg::PoseStamped msg;
  void publish() { publisher->publish(msg); }
};

}  // namespace

class LinktrackNode : public rclcpp::Node {
public:
  LinktrackNode() : Node("linktrack_example") {
    // 参数声明和获取
    this->declare_parameter<std::string>("map_frame", "linktrack_map");
    this->get_parameter("map_frame", frame_id);

    // 初始化订阅和发布结构
    sub_anchorframe0_ = this->create_subscription<uwbpsr_ratros2::msg::LinktrackAnchorframe0>(
      "nlink_linktrack_anchorframe0", 10,
      std::bind(&LinktrackNode::Anchorframe0Callback, this, _1));

    sub_tagframe0_ = this->create_subscription<uwbpsr_ratros2::msg::LinktrackTagframe0>(
      "nlink_linktrack_tagframe0", 10,
      std::bind(&LinktrackNode::Tagframe0Callback, this, _1));

    sub_nodeframe1_ = this->create_subscription<uwbpsr_ratros2::msg::LinktrackNodeframe1>(
      "nlink_linktrack_nodeframe1", 10,
      std::bind(&LinktrackNode::Nodeframe1Callback, this, _1));

    sub_nodeframe2_ = this->create_subscription<uwbpsr_ratros2::msg::LinktrackNodeframe2>(
      "nlink_linktrack_nodeframe2", 10,
      std::bind(&LinktrackNode::Nodeframe2Callback, this, _1));
  }

private:
  std::map<uint8_t, PosePair> poses_anchorframe0_;
  std::map<uint8_t, PosePair> poses_nodeframe1_;

  PosePair pose_tagframe0_;
  PosePair pose_nodeframe2_;

  rclcpp::Subscription<uwbpsr_ratros2::msg::LinktrackAnchorframe0>::SharedPtr sub_anchorframe0_;
  rclcpp::Subscription<uwbpsr_ratros2::msg::LinktrackTagframe0>::SharedPtr sub_tagframe0_;
  rclcpp::Subscription<uwbpsr_ratros2::msg::LinktrackNodeframe1>::SharedPtr sub_nodeframe1_;
  rclcpp::Subscription<uwbpsr_ratros2::msg::LinktrackNodeframe2>::SharedPtr sub_nodeframe2_;

  void Anchorframe0Callback(const uwbpsr_ratros2::msg::LinktrackAnchorframe0::SharedPtr msg) {
    for (const auto &node : msg->nodes) {
      auto id = node.id;
      if (!poses_anchorframe0_.count(id)) {
        std::ostringstream oss;
        oss << "nlt_anchorframe0_pose_node" << static_cast<int>(id);
        auto topic = oss.str();
        poses_anchorframe0_[id].publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);

        auto &msg_pose = poses_anchorframe0_[id].msg;
        msg_pose.header.frame_id = frame_id;
        msg_pose.pose.orientation.w = 0;
        msg_pose.pose.orientation.x = 0;
        msg_pose.pose.orientation.y = 0;
        msg_pose.pose.orientation.z = 1;
      }
      auto &msg_pose = poses_anchorframe0_[id].msg;
      msg_pose.header.stamp = this->now();
      msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
      msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
      msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
      poses_anchorframe0_[id].publish();
    }
  }

  void Nodeframe1Callback(const uwbpsr_ratros2::msg::LinktrackNodeframe1::SharedPtr msg) {
    for (const auto &node : msg->nodes) {
      auto id = node.id;
      if (!poses_nodeframe1_.count(id)) {
        std::ostringstream oss;
        oss << "nlt_nodeframe1_pose_node" << static_cast<int>(id);
        auto topic = oss.str();
        poses_nodeframe1_[id].publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);

        auto &msg_pose = poses_nodeframe1_[id].msg;
        msg_pose.header.frame_id = frame_id;
        msg_pose.pose.orientation.w = 0;
        msg_pose.pose.orientation.x = 0;
        msg_pose.pose.orientation.y = 0;
        msg_pose.pose.orientation.z = 1;
      }
      auto &msg_pose = poses_nodeframe1_[id].msg;
      msg_pose.header.stamp = this->now();
      msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
      msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
      msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
      poses_nodeframe1_[id].publish();
    }
  }

  void Tagframe0Callback(const uwbpsr_ratros2::msg::LinktrackTagframe0::SharedPtr msg) {
    if (!pose_tagframe0_.publisher) {
      pose_tagframe0_.publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("nlt_tagframe0_pose", 10);
      pose_tagframe0_.msg.header.frame_id = frame_id;
    }
    auto &msg_pose = pose_tagframe0_.msg;
    msg_pose.header.stamp = this->now();
    msg_pose.pose.orientation.w = static_cast<double>(msg->quaternion[0]);
    msg_pose.pose.orientation.x = static_cast<double>(msg->quaternion[1]);
    msg_pose.pose.orientation.y = static_cast<double>(msg->quaternion[2]);
    msg_pose.pose.orientation.z = static_cast<double>(msg->quaternion[3]);
    msg_pose.pose.position.x = static_cast<double>(msg->pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(msg->pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(msg->pos_3d[2]);
    pose_tagframe0_.publish();
  }

  void Nodeframe2Callback(const uwbpsr_ratros2::msg::LinktrackNodeframe2::SharedPtr msg) {
    if (!pose_nodeframe2_.publisher) {
      pose_nodeframe2_.publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("nlt_nodeframe2_pose", 10);
      pose_nodeframe2_.msg.header.frame_id = frame_id;
    }
    auto &msg_pose = pose_nodeframe2_.msg;
    msg_pose.header.stamp = this->now();
    msg_pose.pose.orientation.w = static_cast<double>(msg->quaternion[0]);
    msg_pose.pose.orientation.x = static_cast<double>(msg->quaternion[1]);
    msg_pose.pose.orientation.y = static_cast<double>(msg->quaternion[2]);
    msg_pose.pose.orientation.z = static_cast<double>(msg->quaternion[3]);
    msg_pose.pose.position.x = static_cast<double>(msg->pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(msg->pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(msg->pos_3d[2]);
    pose_nodeframe2_.publish();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LinktrackNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
