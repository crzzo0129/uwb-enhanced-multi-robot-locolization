#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <Eigen/Dense>

#include "robot_fusion/ukf.hpp"

using namespace std::chrono_literals;

class RobotUKFNode : public rclcpp::Node {
public:
  explicit RobotUKFNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("ukf_node", options) {
    
    // 参数声明和获取
    std::string robot_name = declare_parameter<std::string>("robot_name", "rb1");
    robot_name_ = robot_name;
    
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/" + robot_name + "/odom");
    
    trilat_topics_.push_back(declare_parameter<std::string>(
        "trilat_topic_other1", "/" + robot_name + "/tri_pos_in_rb_other1"));
    trilat_topics_.push_back(declare_parameter<std::string>(
        "trilat_topic_other2", "/" + robot_name + "/tri_pos_in_rb_other2"));
    
    output_topic_ = declare_parameter<std::string>("output_topic", "/" + robot_name + "/ukf_pose");
    
    update_rate_ = declare_parameter<double>("update_rate", 20.0);
    
    odom_pos_noise_ = declare_parameter<double>("odom_pos_noise", 0.01);
    odom_ang_noise_ = declare_parameter<double>("odom_ang_noise", 0.001);
    trilat_noise_ = declare_parameter<double>("trilat_noise", 0.005);        // 已调整
    process_noise_pos_ = declare_parameter<double>("process_noise_pos", 0.15);
    process_noise_ang_ = declare_parameter<double>("process_noise_ang", 0.02);

    // 初始化 UKF
    ukf_ = std::make_unique<robot_fusion::UKF>(1.0 / update_rate_);

    // 订阅 odom
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&RobotUKFNode::odomCallback, this, std::placeholders::_1));

    // 订阅 trilateration
    trilat_subs_.resize(2);
    for (size_t i = 0; i < 2; ++i) {
      trilat_subs_[i] = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          trilat_topics_[i], 10,
          [this, i](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            trilatCallback(msg, i);
          });
    }

    // 订阅其他机器人 UKF pose（带平滑）
    other_pose_subs_.resize(2);
    for (size_t i = 0; i < 2; ++i) {
      std::string other_name = (i == 0) ? "rb2" : "rb3";
      other_pose_subs_[i] = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/" + other_name + "/ukf_pose", 10,
          [this, i](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            otherPoseCallback(msg, i);
          });
    }

    // Publisher
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        output_topic_, 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Timer for prediction
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
        std::bind(&RobotUKFNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "UKF node for %s initialized", robot_name_.c_str());
  }

private:
  std::string robot_name_;
  std::string odom_topic_;
  std::vector<std::string> trilat_topics_;
  std::string output_topic_;
  double update_rate_;

  double odom_pos_noise_;
  double odom_ang_noise_;
  double trilat_noise_;
  double process_noise_pos_;
  double process_noise_ang_;

  std::unique_ptr<robot_fusion::UKF> ukf_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> trilat_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> other_pose_subs_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Other robots' poses with smoothing
  Eigen::Vector3d other_poses_[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  bool other_poses_received_[2] = {false, false};

  rclcpp::TimerBase::SharedPtr timer_;   // ← 修复：timer_ 定义在这里

  bool is_initialized_ = false;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!is_initialized_) {
      initializeUKF(msg);
      is_initialized_ = true;
      return;
    }

    Eigen::Vector3d z_odom;
    z_odom(0) = msg->pose.pose.position.x;
    z_odom(1) = msg->pose.pose.position.y;

    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                               msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    z_odom(2) = std::atan2(siny_cosp, cosy_cosp);

    ukf_->updateOdom(z_odom);
  }

  void otherPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, size_t index) {
    Eigen::Vector3d new_pose;
    new_pose(0) = msg->pose.pose.position.x;
    new_pose(1) = msg->pose.pose.position.y;

    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                         msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                               msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    new_pose(2) = std::atan2(siny, cosy);

    // 指数平滑
    other_poses_[index] = 0.7 * other_poses_[index] + 0.3 * new_pose;
    other_poses_received_[index] = true;
  }

  void trilatCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, size_t index) {
    if (!other_poses_received_[index]) return;

    double x_rel = msg->pose.pose.position.x;
    double y_rel = msg->pose.pose.position.y;
    double measured_range = std::sqrt(x_rel*x_rel + y_rel*y_rel);

    Eigen::Vector2d other_pos = other_poses_[index].head<2>();

    ukf_->updateTrilatMeasurement(measured_range, other_pos, trilat_noise_);

    double innovation = std::abs(measured_range - (ukf_->getState().head<2>() - other_pos).norm());
    RCLCPP_INFO(get_logger(), "[%s] Range-only[%zu] measured=%.3fm  innovation=%.4fm",
                robot_name_.c_str(), index, measured_range, innovation);
  }

  void timerCallback() {
    ukf_->predict(1.0 / update_rate_);
    publishPose();
  }

  void initializeUKF(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    x0(0) = msg->pose.pose.position.x;
    x0(1) = msg->pose.pose.position.y;
    
    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                               msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    x0(2) = std::atan2(siny_cosp, cosy_cosp);
    
    if (msg->twist.twist.linear.x != 0 || msg->twist.twist.linear.y != 0) {
      x0(3) = msg->twist.twist.linear.x;
      x0(4) = msg->twist.twist.linear.y;
    }
    x0(5) = msg->twist.twist.angular.z;

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(6, 6);
    P0.diagonal() << odom_pos_noise_, odom_pos_noise_, odom_ang_noise_, 0.1, 0.1, 0.01;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    Q.diagonal() << process_noise_pos_, process_noise_pos_, process_noise_ang_,
                    process_noise_pos_, process_noise_pos_, process_noise_ang_;

    Eigen::MatrixXd R_odom = Eigen::MatrixXd::Zero(3, 3);
    R_odom.diagonal() << odom_pos_noise_, odom_pos_noise_, odom_ang_noise_;

    ukf_->initialize(x0, P0, Q, R_odom);

    RCLCPP_INFO(get_logger(), "UKF initialized for %s at (%.3f, %.3f, %.3f)",
                robot_name_.c_str(), x0(0), x0(1), x0(2));
  }

  void publishPose() {
    auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = robot_name_ + "/odom";

    const auto& state = ukf_->getState();
    const auto& P = ukf_->getCovariance();

    msg->pose.pose.position.x = state(0);
    msg->pose.pose.position.y = state(1);
    msg->pose.pose.position.z = 0.0;

    double yaw = state(2);
    msg->pose.pose.orientation.w = std::cos(yaw / 2.0);
    msg->pose.pose.orientation.z = std::sin(yaw / 2.0);

    msg->pose.covariance[0]  = P(0, 0);
    msg->pose.covariance[1]  = P(0, 1);
    msg->pose.covariance[6]  = P(1, 0);
    msg->pose.covariance[7]  = P(1, 1);
    msg->pose.covariance[35] = P(2, 2);

    pose_pub_->publish(std::move(msg));
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotUKFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}