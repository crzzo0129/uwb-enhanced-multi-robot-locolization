#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace gazebo
{
class RobotDistancePlugin : public WorldPlugin
{
public:
  RobotDistancePlugin() {}

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    // 检查 ROS 2 是否已初始化，若未初始化则初始化
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // 创建 ROS 2 节点
    node_ = rclcpp::Node::make_shared("robot_distance_plugin");

    // 创建发布器
    distance_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_distances", 30);

    // 保存世界指针
    world_ = _world;
    // 读取 <update_rate>，默认 20.0 Hz
    update_rate_ = 20.0;
    if (_sdf->HasElement("update_rate"))
      update_rate_ = _sdf->Get<double>("update_rate");

    last_pub_time_ = world_->SimTime();

    // 注册更新回调
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RobotDistancePlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "RobotDistancePlugin loaded");
  }

  ~RobotDistancePlugin()
  {
    // 仅在插件卸载时关闭 ROS 2（如果需要）
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

private:
  void OnUpdate()
  {
    const common::Time now = world_->SimTime();
    const double dt = (now - last_pub_time_).Double();
    if (dt < (1.0 / update_rate_)) return;     // 仿真时间节流
    last_pub_time_ = now;

    // 获取三个机器人的模型
    physics::ModelPtr rb1 = world_->ModelByName("robot1");
    physics::ModelPtr rb2 = world_->ModelByName("robot2");
    physics::ModelPtr rb3 = world_->ModelByName("robot3");

    if (!rb1 || !rb2 || !rb3)
    {
      RCLCPP_ERROR(node_->get_logger(), "One or more robots (rb1, rb2, rb3) not found in the world!");
      return;
    }

    // 获取机器人位置
    ignition::math::Pose3d pose1 = rb1->WorldPose();
    ignition::math::Pose3d pose2 = rb2->WorldPose();
    ignition::math::Pose3d pose3 = rb3->WorldPose();

    // 计算两两之间的距离
    double dist_12 = pose1.Pos().Distance(pose2.Pos());
    double dist_13 = pose1.Pos().Distance(pose3.Pos());
    double dist_23 = pose2.Pos().Distance(pose3.Pos());

    // 发布距离信息
    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "world";
    msg->pose.position.x = dist_12;
    msg->pose.position.y = dist_13;
    msg->pose.position.z = dist_23;
    distance_pub_->publish(std::move(msg));

    //RCLCPP_INFO_STREAM(node_->get_logger(), "Distances: rb1-rb2=" << dist_12 << ", rb1-rb3=" << dist_13 << ", rb2-rb3=" << dist_23);
  }

  // 成员变量
  physics::WorldPtr world_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr distance_pub_;
  event::ConnectionPtr update_connection_;
  common::Time last_pub_time_;
  double update_rate_;
};

GZ_REGISTER_WORLD_PLUGIN(RobotDistancePlugin)
} // namespace gazebo
