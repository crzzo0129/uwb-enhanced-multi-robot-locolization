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
    
    // Declare and get parameters
    std::string robot_name = declare_parameter<std::string>("robot_name", "rb1");
    robot_name_ = robot_name;
    
    odom_topic_ = declare_parameter<std::string>(
        "odom_topic", "/" + robot_name + "/odom");
    
    // Trilateration topics from other robots
    trilat_topics_.push_back(declare_parameter<std::string>(
        "trilat_topic_other1", "/" + robot_name + "/tri_pos_in_rb_other1"));
    trilat_topics_.push_back(declare_parameter<std::string>(
        "trilat_topic_other2", "/" + robot_name + "/tri_pos_in_rb_other2"));
    
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/" + robot_name + "/ukf_pose");
    
    // Initialize frequency
    update_rate_ = declare_parameter<double>("update_rate", 20.0);
    
    // Get noise parameters
    odom_pos_noise_ = declare_parameter<double>("odom_pos_noise", 0.01);
    odom_ang_noise_ = declare_parameter<double>("odom_ang_noise", 0.001);
    trilat_noise_ = declare_parameter<double>("trilat_noise", 0.0225);
    
    // Process noise
    process_noise_pos_ = declare_parameter<double>("process_noise_pos", 0.01);
    process_noise_ang_ = declare_parameter<double>("process_noise_ang", 0.001);
    
    // Initialize UKF
    ukf_ = std::make_unique<robot_fusion::UKF>(1.0 / update_rate_);
    
    // Set up subscriptions
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&RobotUKFNode::odomCallback, this, std::placeholders::_1));
    
    trilat_subs_.resize(2);
    for (size_t i = 0; i < trilat_subs_.size(); ++i) {
      trilat_subs_[i] = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          trilat_topics_[i], 10,
          [this, i](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            trilatCallback(msg, i);
          });
    }
    
    // Publisher for UKF pose
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
  
  // Noise parameters
  double odom_pos_noise_;
  double odom_ang_noise_;
  double trilat_noise_;
  double process_noise_pos_;
  double process_noise_ang_;
  
  // UKF instance
  std::unique_ptr<robot_fusion::UKF> ukf_;
  
  // Subscribers and publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> trilat_subs_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Last odom time for initialization
  rclcpp::Time last_odom_time_;
  bool is_initialized_ = false;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!is_initialized_) {
      // Initialize UKF with first odometry measurement
      initializeUKF(msg);
      is_initialized_ = true;
      return;
    }
    
    // Update measurement
    Eigen::Vector3d z_odom;
    z_odom(0) = msg->pose.pose.position.x;
    z_odom(1) = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                               msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    z_odom(2) = std::atan2(siny_cosp, cosy_cosp);
    
    // Update filter with odometry
    ukf_->updateOdom(z_odom);
    
    last_odom_time_ = msg->header.stamp;
  }

  void trilatCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
      size_t index) {
    // Update with trilateration measurement
    Eigen::Vector2d z_trilat;
    z_trilat(0) = msg->pose.pose.position.x;
    z_trilat(1) = msg->pose.pose.position.y;
    
    Eigen::Matrix2d trilat_noise = Eigen::Matrix2d::Identity() * trilat_noise_;
    ukf_->updateTrilatMeasurement(z_trilat, trilat_noise);
  }

  void timerCallback() {
    // Prediction step
    double dt = 1.0 / update_rate_;
    ukf_->predict(dt);
    
    // Publish current estimate
    publishPose();
  }

  void initializeUKF(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Initialize state
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    x0(0) = msg->pose.pose.position.x;
    x0(1) = msg->pose.pose.position.y;
    
    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                               msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    x0(2) = std::atan2(siny_cosp, cosy_cosp);
    
    // Extract velocity from odometry if available
    if (msg->twist.twist.linear.x != 0 || msg->twist.twist.linear.y != 0) {
      x0(3) = msg->twist.twist.linear.x;
      x0(4) = msg->twist.twist.linear.y;
    }
    x0(5) = msg->twist.twist.angular.z;
    
    // Initialize covariance matrices
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(6, 6);
    P0(0, 0) = odom_pos_noise_;
    P0(1, 1) = odom_pos_noise_;
    P0(2, 2) = odom_ang_noise_;
    P0(3, 3) = 0.1;  // velocity uncertainty
    P0(4, 4) = 0.1;
    P0(5, 5) = 0.01;  // angular velocity uncertainty
    
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    Q(0, 0) = process_noise_pos_;
    Q(1, 1) = process_noise_pos_;
    Q(2, 2) = process_noise_ang_;
    Q(3, 3) = process_noise_pos_;
    Q(4, 4) = process_noise_pos_;
    Q(5, 5) = process_noise_ang_;
    
    Eigen::MatrixXd R_odom = Eigen::MatrixXd::Zero(3, 3);
    R_odom(0, 0) = odom_pos_noise_;
    R_odom(1, 1) = odom_pos_noise_;
    R_odom(2, 2) = odom_ang_noise_;
    
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
    
    // Set pose
    msg->pose.pose.position.x = state(0);
    msg->pose.pose.position.y = state(1);
    msg->pose.pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    double yaw = state(2);
    msg->pose.pose.orientation.w = std::cos(yaw / 2.0);
    msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
    
    // Set covariance
    msg->pose.covariance[0] = P(0, 0);   // x variance
    msg->pose.covariance[1] = P(0, 1);   // x-y covariance
    msg->pose.covariance[6] = P(1, 0);   // y-x covariance
    msg->pose.covariance[7] = P(1, 1);   // y variance
    msg->pose.covariance[35] = P(2, 2);  // theta variance
    
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
