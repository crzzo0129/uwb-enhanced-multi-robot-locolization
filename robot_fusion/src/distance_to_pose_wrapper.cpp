#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>

class DistanceToPoseWrapper : public rclcpp::Node {
public:
  DistanceToPoseWrapper() : Node("distance_to_pose_wrapper"), tf_available_(true) {
    // 订阅 /robot_distances 话题
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_distances", 10,
        std::bind(&DistanceToPoseWrapper::distance_callback, this, std::placeholders::_1));

    // 为每个机器人对创建发布器
    pubs_["rb1_rb2"] = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rb1/relative_pose_rb2", 10);
    pubs_["rb1_rb3"] = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rb1/relative_pose_rb3", 10);
    pubs_["rb2_rb1"] = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rb2/relative_pose_rb1", 10);
    pubs_["rb2_rb3"] = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rb2/relative_pose_rb3", 10);
    pubs_["rb3_rb1"] = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rb3/relative_pose_rb1", 10);
    pubs_["rb3_rb2"] = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rb3/relative_pose_rb2", 10);

    // 初始化 TF 监听器（保留以防未来扩展）
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // UWB 噪声协方差
    uwb_variance_ = 0.0225;  // (0.15 m)^2
    theta_variance_ = 0.01;  // 角度协方差 (rad^2)

    // 直接设置 tf_available_ 为 true，无需检查 map
    RCLCPP_INFO(get_logger(), "DistanceToPoseWrapper initialized, ready to process /robot_distances");
  }

  ~DistanceToPoseWrapper() {}

private:
  void distance_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 提取三条边的距离 (假设 msg->pose.position.x/y/z 包含 d12, d13, d23)
    double d12 = msg->pose.position.x;  // rb1 到 rb2 的距离
    double d13 = msg->pose.position.y;  // rb1 到 rb3 的距离
    double d23 = msg->pose.position.z;  // rb2 到 rb3 的距离

    // 验证距离有效性
    if (d12 <= 0 || d13 <= 0 || d23 <= 0) {
      RCLCPP_WARN(get_logger(), "Invalid distances received: d12=%f, d13=%f, d23=%f", d12, d13, d23);
      return;
    }

    // 检查三角形不等式
    if (d12 + d13 <= d23 || d12 + d23 <= d13 || d13 + d23 <= d12) {
      RCLCPP_WARN(get_logger(), "Distances violate triangle inequality: d12=%f, d13=%f, d23=%f", d12, d13, d23);
      return;
    }

    // 1. rb1 视角: rb2 和 rb3 相对 rb1
    double cos_alpha = (d12 * d12 + d13 * d13 - d23 * d23) / (2 * d12 * d13);
    if (d12 * d13 == 0) cos_alpha = 0;
    cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
    double alpha = std::acos(cos_alpha);

    // rb2 相对于 rb1
    geometry_msgs::msg::PoseWithCovarianceStamped rel_pose_rb1_rb2;
    rel_pose_rb1_rb2.header.stamp = now();
    rel_pose_rb1_rb2.header.frame_id = "rb1/base_link";
    rel_pose_rb1_rb2.pose.pose.position.x = d12 * std::cos(alpha / 2);
    rel_pose_rb1_rb2.pose.pose.position.y = d12 * std::sin(alpha / 2);
    rel_pose_rb1_rb2.pose.pose.orientation.z = std::sin(alpha / 2);
    rel_pose_rb1_rb2.pose.pose.orientation.w = std::cos(alpha / 2);
    rel_pose_rb1_rb2.pose.covariance = {
        uwb_variance_, 0, 0, 0, 0, 0,
        0, uwb_variance_, 0, 0, 0, 0,
        0, 0, 1e9, 0, 0, 0,
        0, 0, 0, 1e9, 0, 0,
        0, 0, 0, 0, 1e9, 0,
        0, 0, 0, 0, 0, theta_variance_
    };
    pubs_["rb1_rb2"]->publish(rel_pose_rb1_rb2);

    // rb3 相对于 rb1
    geometry_msgs::msg::PoseWithCovarianceStamped rel_pose_rb1_rb3;
    rel_pose_rb1_rb3.header.stamp = now();
    rel_pose_rb1_rb3.header.frame_id = "rb1/base_link";
    rel_pose_rb1_rb3.pose.pose.position.x = d13 * std::cos(-alpha / 2);
    rel_pose_rb1_rb3.pose.pose.position.y = d13 * std::sin(-alpha / 2);
    rel_pose_rb1_rb3.pose.pose.orientation.z = std::sin(-alpha / 2);
    rel_pose_rb1_rb3.pose.pose.orientation.w = std::cos(-alpha / 2);
    rel_pose_rb1_rb3.pose.covariance = {
        uwb_variance_, 0, 0, 0, 0, 0,
        0, uwb_variance_, 0, 0, 0, 0,
        0, 0, 1e9, 0, 0, 0,
        0, 0, 0, 1e9, 0, 0,
        0, 0, 0, 0, 1e9, 0,
        0, 0, 0, 0, 0, theta_variance_
    };
    pubs_["rb1_rb3"]->publish(rel_pose_rb1_rb3);

    // 2. rb2 视角: rb1 和 rb3 相对 rb2
    double cos_beta = (d12 * d12 + d23 * d23 - d13 * d13) / (2 * d12 * d23);
    if (d12 * d23 == 0) cos_beta = 0;
    cos_beta = std::clamp(cos_beta, -1.0, 1.0);
    double beta = std::acos(cos_beta);

    // rb1 相对于 rb2
    geometry_msgs::msg::PoseWithCovarianceStamped rel_pose_rb2_rb1;
    rel_pose_rb2_rb1.header.stamp = now();
    rel_pose_rb2_rb1.header.frame_id = "rb2/base_link";
    rel_pose_rb2_rb1.pose.pose.position.x = d12 * std::cos(beta / 2 + M_PI);
    rel_pose_rb2_rb1.pose.pose.position.y = d12 * std::sin(beta / 2 + M_PI);
    rel_pose_rb2_rb1.pose.pose.orientation.z = std::sin(beta / 2 + M_PI);
    rel_pose_rb2_rb1.pose.pose.orientation.w = std::cos(beta / 2 + M_PI);
    rel_pose_rb2_rb1.pose.covariance = {
        uwb_variance_, 0, 0, 0, 0, 0,
        0, uwb_variance_, 0, 0, 0, 0,
        0, 0, 1e9, 0, 0, 0,
        0, 0, 0, 1e9, 0, 0,
        0, 0, 0, 0, 1e9, 0,
        0, 0, 0, 0, 0, theta_variance_
    };
    pubs_["rb2_rb1"]->publish(rel_pose_rb2_rb1);

    // rb3 相对于 rb2
    geometry_msgs::msg::PoseWithCovarianceStamped rel_pose_rb2_rb3;
    rel_pose_rb2_rb3.header.stamp = now();
    rel_pose_rb2_rb3.header.frame_id = "rb2/base_link";
    rel_pose_rb2_rb3.pose.pose.position.x = d23 * std::cos(-beta / 2);
    rel_pose_rb2_rb3.pose.pose.position.y = d23 * std::sin(-beta / 2);
    rel_pose_rb2_rb3.pose.pose.orientation.z = std::sin(-beta / 2);
    rel_pose_rb2_rb3.pose.pose.orientation.w = std::cos(-beta / 2);
    rel_pose_rb2_rb3.pose.covariance = {
        uwb_variance_, 0, 0, 0, 0, 0,
        0, uwb_variance_, 0, 0, 0, 0,
        0, 0, 1e9, 0, 0, 0,
        0, 0, 0, 1e9, 0, 0,
        0, 0, 0, 0, 1e9, 0,
        0, 0, 0, 0, 0, theta_variance_
    };
    pubs_["rb2_rb3"]->publish(rel_pose_rb2_rb3);

    // 3. rb3 视角: rb1 和 rb2 相对 rb3
    double cos_gamma = (d13 * d13 + d23 * d23 - d12 * d12) / (2 * d13 * d23);
    if (d13 * d23 == 0) cos_gamma = 0;
    cos_gamma = std::clamp(cos_gamma, -1.0, 1.0);
    double gamma = std::acos(cos_gamma);

    // rb1 相对于 rb3
    geometry_msgs::msg::PoseWithCovarianceStamped rel_pose_rb3_rb1;
    rel_pose_rb3_rb1.header.stamp = now();
    rel_pose_rb3_rb1.header.frame_id = "rb3/base_link";
    rel_pose_rb3_rb1.pose.pose.position.x = d13 * std::cos(gamma / 2 + M_PI);
    rel_pose_rb3_rb1.pose.pose.position.y = d13 * std::sin(gamma / 2 + M_PI);
    rel_pose_rb3_rb1.pose.pose.orientation.z = std::sin(gamma / 2 + M_PI);
    rel_pose_rb3_rb1.pose.pose.orientation.w = std::cos(gamma / 2 + M_PI);
    rel_pose_rb3_rb1.pose.covariance = {
        uwb_variance_, 0, 0, 0, 0, 0,
        0, uwb_variance_, 0, 0, 0, 0,
        0, 0, 1e9, 0, 0, 0,
        0, 0, 0, 1e9, 0, 0,
        0, 0, 0, 0, 1e9, 0,
        0, 0, 0, 0, 0, theta_variance_
    };
    pubs_["rb3_rb1"]->publish(rel_pose_rb3_rb1);

    // rb2 相对于 rb3
    geometry_msgs::msg::PoseWithCovarianceStamped rel_pose_rb3_rb2;
    rel_pose_rb3_rb2.header.stamp = now();
    rel_pose_rb3_rb2.header.frame_id = "rb3/base_link";
    rel_pose_rb3_rb2.pose.pose.position.x = d23 * std::cos(-gamma / 2);
    rel_pose_rb3_rb2.pose.pose.position.y = d23 * std::sin(-gamma / 2);
    rel_pose_rb3_rb2.pose.pose.orientation.z = std::sin(-gamma / 2);
    rel_pose_rb3_rb2.pose.pose.orientation.w = std::cos(-gamma / 2);
    rel_pose_rb3_rb2.pose.covariance = {
        uwb_variance_, 0, 0, 0, 0, 0,
        0, uwb_variance_, 0, 0, 0, 0,
        0, 0, 1e9, 0, 0, 0,
        0, 0, 0, 1e9, 0, 0,
        0, 0, 0, 0, 1e9, 0,
        0, 0, 0, 0, 0, theta_variance_
    };
    pubs_["rb3_rb2"]->publish(rel_pose_rb3_rb2);

    RCLCPP_DEBUG(get_logger(), "Published relative poses for all robot pairs");
  }

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pubs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool tf_available_;
  double uwb_variance_;
  double theta_variance_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistanceToPoseWrapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}