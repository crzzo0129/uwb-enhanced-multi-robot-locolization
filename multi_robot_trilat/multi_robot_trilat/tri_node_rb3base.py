# tri_node_rb3base.py
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster


def rot2d(theta: float) -> np.ndarray:
    c = math.cos(theta); s = math.sin(theta)
    return np.array([[c, -s],[s, c]], dtype=float)

def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def trilat_candidates_solver_frame(d23: float, d21: float, d13: float):
    """对应新的角色，d23=rb2-rb3, d21=rb2-rb1, d13=rb1-rb3"""
    d13_safe = max(d13, 1e-9)
    x = (d23**2 + d13_safe**2 - d21**2) / (2.0 * d13_safe)
    y2 = max(0.0, d23**2 - x**2)
    y = math.sqrt(y2)
    return np.array([x, +y], float), np.array([x, -y], float)


class TrilatNodeRB3(Node):
    def __init__(self):
        super().__init__('trilat_node_rb3base')

        # 话题
        self.distances_topic = self.declare_parameter('distances_topic', '/robot_distances').value
        self.rb1_odom_topic  = self.declare_parameter('rb1_odom_topic', '/rb1/odom').value
        self.rb2_odom_topic  = self.declare_parameter('rb2_odom_topic', '/rb2/odom').value
        self.rb3_odom_topic  = self.declare_parameter('rb3_odom_topic', '/rb3/odom').value
        self.rb3_imu_topic   = self.declare_parameter('rb3_imu_topic',  '/rb3/imu_plugin/out').value

        # 发布：在 rb3 基准下，输出 rb2 和 rb1 的位置
        self.pub_pose2_topic  = self.declare_parameter('pub_pose2_topic',  '/rb2/tri_pos_in_rb3').value
        self.pub_pose1_topic  = self.declare_parameter('pub_pose1_topic',  '/rb1/tri_pos_in_rb3').value
        self.pub_tf          = self.declare_parameter('pub_tf', True).value
        self.rb3_frame_id    = self.declare_parameter('rb3_frame_id', 'rb3/base_link').value
        self.rb2_child_frame_id = self.declare_parameter('rb2_child_frame_id', 'rb2/tri_estimate').value
        self.rb1_child_frame_id = self.declare_parameter('rb1_child_frame_id', 'rb1/tri_estimate').value

        # 参数
        self.init_known = self.declare_parameter('init_known', True).value
        self.init_rb1_xy = np.array(self.declare_parameter('init_rb1_xy', [0.0, 0.0]).value, float)
        self.init_rb2_xy = np.array(self.declare_parameter('init_rb2_xy', [0.0, 2.0]).value, float)
        self.init_rb3_xy = np.array(self.declare_parameter('init_rb3_xy', [1.0, 1.0]).value, float)
        self.init_rb3_yaw_deg = float(self.declare_parameter('init_rb3_yaw_deg', 0.0).value)

        # 分支迟滞与死区
        self.flip_hysteresis = int(self.declare_parameter('flip_hysteresis', 2).value)
        self.y_deadzone = float(self.declare_parameter('y_deadzone', 0.02).value)
        self._flip_count = 0
        self.branch_sign = None

        # QoS
        odom_qos = QoSProfile(depth=50)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE
        odom_qos.history = HistoryPolicy.KEEP_LAST

        self.sub_dist = self.create_subscription(PoseStamped, self.distances_topic, self.on_dist, 10)
        self.sub_odom1 = self.create_subscription(Odometry, self.rb1_odom_topic, self.on_odom1, odom_qos)
        self.sub_odom2 = self.create_subscription(Odometry, self.rb2_odom_topic, self.on_odom2, odom_qos)
        self.sub_odom3 = self.create_subscription(Odometry, self.rb3_odom_topic, self.on_odom3, odom_qos)
        self.sub_imu3  = self.create_subscription(Imu,      self.rb3_imu_topic,  self.on_imu3,  qos_profile_sensor_data)

        self.pub_pose2 = self.create_publisher(PoseWithCovarianceStamped, self.pub_pose2_topic, 10)
        self.pub_pose1 = self.create_publisher(PoseWithCovarianceStamped, self.pub_pose1_topic, 10)
        self.br = TransformBroadcaster(self) if self.pub_tf else None

        # 最新里程计
        self.p1 = None; self.p2 = None; self.p3 = None
        self.yaw3 = None
        self.pos2_rb3 = None
        self.pos1_rb3 = None

    def on_odom1(self, msg: Odometry):
        self.p1 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], float)

    def on_odom2(self, msg: Odometry):
        self.p2 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], float)

    def on_odom3(self, msg: Odometry):
        self.p3 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], float)
        self.yaw3 = yaw_from_quat(msg.pose.pose.orientation)

    def on_imu3(self, msg: Imu):
        pass

    def on_dist(self, msg: PoseStamped):
        d12 = float(msg.pose.position.x)  # d12: rb1-rb2
        d13 = float(msg.pose.position.y)  # d13: rb1-rb3
        d23 = float(msg.pose.position.z)  # d23: rb2-rb3

        if self.p1 is None or self.p2 is None or self.p3 is None or self.yaw3 is None:
            return

        # 世界 -> rb3
        R_w2b = rot2d(-self.yaw3)
        r13_rb3 = R_w2b @ (self.p1 - self.p3)   # rb1 在 rb3 下
        r23_rb3 = R_w2b @ (self.p2 - self.p3)   # rb2 在 rb3 下

        # 解算系：rb1 在 x 轴
        theta = math.atan2(r13_rb3[1], r13_rb3[0])
        Rs = rot2d(-theta); Rt = rot2d(+theta)

        # 侧别判据：rb2 在解算系的 y 符号
        r23_s = Rs @ r23_rb3
        desired_branch = self.branch_sign
        if   r23_s[1] >  self.y_deadzone: desired_branch = +1
        elif r23_s[1] < -self.y_deadzone: desired_branch = -1
        if self.branch_sign is None:
            self.branch_sign = desired_branch if desired_branch is not None else +1
        elif desired_branch is not None and desired_branch != self.branch_sign:
            self._flip_count += 1
            if self._flip_count >= self.flip_hysteresis:
                self.branch_sign = desired_branch
                self._flip_count = 0
        else:
            self._flip_count = 0

        # 两解，选择
        p_plus_s, p_minus_s = trilat_candidates_solver_frame(d23, d21=d12, d13=d13)
        cand_s = p_plus_s if self.branch_sign > 0 else p_minus_s

        self.pos2_rb3 = Rt @ cand_s
        self.pos1_rb3 = Rt @ np.array([d13, 0.0], float)

        self.publish_pose2(msg.header.stamp, self.pos2_rb3)
        self.publish_pose1(msg.header.stamp, self.pos1_rb3)

    def publish_pose2(self, stamp, pos_rb3_xy: np.ndarray):
        self._publish_pose(self.pub_pose2, self.rb3_frame_id, self.rb2_child_frame_id, stamp, pos_rb3_xy)

    def publish_pose1(self, stamp, pos_rb3_xy: np.ndarray):
        self._publish_pose(self.pub_pose1, self.rb3_frame_id, self.rb1_child_frame_id, stamp, pos_rb3_xy)

    def _publish_pose(self, pub, frame_id, child_id, stamp, pos_xy):
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        out.pose.pose = Pose(
            position=Point(x=float(pos_xy[0]), y=float(pos_xy[1]), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        cov = [0.0]*36
        cov[0] = 0.05; cov[7] = 0.05
        cov[14] = cov[21] = cov[28] = cov[35] = 1e6
        out.pose.covariance = cov
        pub.publish(out)

        if self.pub_tf and self.br is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = frame_id
            tf.child_frame_id = child_id
            tf.transform.translation.x = float(pos_xy[0])
            tf.transform.translation.y = float(pos_xy[1])
            tf.transform.translation.z = 0.0
            tf.transform.rotation.w = 1.0
            self.br.sendTransform(tf)


def main():
    rclpy.init()
    node = TrilatNodeRB3()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
