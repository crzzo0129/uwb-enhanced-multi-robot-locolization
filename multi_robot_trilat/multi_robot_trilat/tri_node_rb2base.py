import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def rot2d(theta: float) -> np.ndarray:
    c = math.cos(theta); s = math.sin(theta)
    return np.array([[c, -s],[s, c]], dtype=float)

def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def trilat_candidates_solver_frame(d12: float, d13: float, d23: float):
    d23_safe = max(d23, 1e-9)
    x = (d12**2 + d23_safe**2 - d13**2) / (2.0 * d23_safe)
    y2 = max(0.0, d12**2 - x**2)
    y = math.sqrt(y2)
    return np.array([x, +y], float), np.array([x, -y], float)


class TrilatNode(Node):
    """
    关键改动：
    1) 每帧用 /rb1/odom, /rb2/odom, /rb3/odom 直接计算 r32 = R(-yaw2)*(p3 - p2) 与 r12 = R(-yaw2)*(p1 - p2)
       由此得到解算系旋转角 theta 以及“侧别符号” sign_y = sign( (R(-theta) r12).y )。
    2) 镜像分支按 sign_y 选择，并加极小死区与迟滞。彻底消除“穿线不翻”的不可观问题。
    3) 不再用只依赖 rb2 的积分推进 prev_pos3,rb3 运动时输出不会乱跳。
    """

    def __init__(self):
        super().__init__('trilat_node')

        # 话题
        self.distances_topic = self.declare_parameter('distances_topic', '/robot_distances').value
        self.rb1_odom_topic  = self.declare_parameter('rb1_odom_topic', '/rb1/odom').value
        self.rb2_odom_topic  = self.declare_parameter('rb2_odom_topic', '/rb2/odom').value
        self.rb3_odom_topic  = self.declare_parameter('rb3_odom_topic', '/rb3/odom').value
        self.rb2_imu_topic   = self.declare_parameter('rb2_imu_topic',  '/rb2/imu_plugin/out').value

        self.pub_pose1_topic  = self.declare_parameter('pub_pose1_topic',  '/rb1/tri_pos_in_rb2').value
        self.pub_pose3_topic  = self.declare_parameter('pub_pose3_topic',  '/rb3/tri_pos_in_rb2').value
        self.pub_tf          = self.declare_parameter('pub_tf', True).value
        self.rb2_frame_id    = self.declare_parameter('rb2_frame_id', 'rb2/base_link').value
        self.rb1_child_frame_id = self.declare_parameter('rb1_child_frame_id', 'rb1/tri_estimate_in_rb2').value
        self.rb3_child_frame_id = self.declare_parameter('rb3_child_frame_id', 'rb3/tri_estimate_in_rb2').value

        # 已知初始（用于首帧兜底；运行期我们每帧用里程计重算 r32、r12）
        self.init_known = self.declare_parameter('init_known', True).value
        self.init_rb1_xy = np.array(self.declare_parameter('init_rb1_xy', [0.0, 0.0]).value, float)
        self.init_rb2_xy = np.array(self.declare_parameter('init_rb2_xy', [0.0, 2.0]).value, float)
        self.init_rb3_xy = np.array(self.declare_parameter('init_rb3_xy', [1.0, 1.0]).value, float)
        self.init_rb2_yaw_deg = float(self.declare_parameter('init_rb2_yaw_deg', 0.0).value)
        self.init_rb2_yaw = math.radians(self.init_rb2_yaw_deg)

        # 分支迟滞与死区
        self.flip_hysteresis = int(self.declare_parameter('flip_hysteresis', 2).value)
        self.y_deadzone = float(self.declare_parameter('y_deadzone', 0.02).value)  # 米
        self._flip_count = 0
        self.branch_sign = None  # +1/-1

        # 订阅 QoS：里程计显式 RELIABLE；IMU 用 sensor QoS
        odom_qos = QoSProfile(depth=50)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE
        odom_qos.history = HistoryPolicy.KEEP_LAST

        self.sub_dist = self.create_subscription(PoseStamped, self.distances_topic, self.on_dist, 10)
        self.sub_odom1 = self.create_subscription(Odometry, self.rb1_odom_topic, self.on_odom1, odom_qos)
        self.sub_odom2 = self.create_subscription(Odometry, self.rb2_odom_topic, self.on_odom2, odom_qos)
        self.sub_odom3 = self.create_subscription(Odometry, self.rb3_odom_topic, self.on_odom3, odom_qos)
        self.sub_imu2  = self.create_subscription(Imu,      self.rb2_imu_topic,  self.on_imu2,  qos_profile_sensor_data)

        self.pub_pose1 = self.create_publisher(PoseWithCovarianceStamped, self.pub_pose1_topic, 10)
        self.pub_pose3 = self.create_publisher(PoseWithCovarianceStamped, self.pub_pose3_topic, 10)
        self.br = TransformBroadcaster(self) if self.pub_tf else None

        # 最新里程计（世界/odom系）
        self.p1 = None  # np.array([x,y]); yaw1 可选
        self.p2 = None
        self.p3 = None
        self.yaw2 = None  # rb2 的朝向（用于世界->rb2 旋转）
        # 估计结果（rb2/base_link）
        self.pos1_rb2 = None
        self.pos3_rb2 = None

        # 首帧时间对齐
        self.last_process_time = None

    # ---------- 里程计/IMU回调 ----------

    def on_odom1(self, msg: Odometry):
        self.p1 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], float)

    def on_odom2(self, msg: Odometry):
        self.p2 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], float)
        self.yaw2 = yaw_from_quat(msg.pose.pose.orientation)

    def on_odom3(self, msg: Odometry):
        self.p3 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y], float)

    def on_imu2(self, msg: Imu):
        # 仅作兜底。如果里程计 yaw2 不可靠，可改用 IMU。
        pass

    # ---------- 距离驱动主流程 ----------

    def on_dist(self, msg: PoseStamped):
        d12 = float(msg.pose.position.x)
        d13 = float(msg.pose.position.y)
        d23 = float(msg.pose.position.z)

        # 确保有 rb1/rb2/rb3 的里程计
        if self.p1 is None or self.p2 is None or self.p3 is None:
            # 还没拿到完整里程计，做一次“已知初始”的兜底
            if self.pos1_rb2 is None and self.init_known:
                self.pos1_rb2 = self.init_first_frame(d12, d13, d23)
                self.publish_pose1(msg.header.stamp, self.pos1_rb2)
                self.last_process_time = Time.from_msg(msg.header.stamp)
            return

        if self.yaw2 is None:
            # 没有 rb2 朝向就无法把世界旋到 rb2
            return

        # 1) 世界 -> rb2/base_link
        R_w2b = rot2d(-self.yaw2)
        r32_rb2 = R_w2b @ (self.p3 - self.p2)   # rb3 在 rb2 下
        r12_rb2 = R_w2b @ (self.p1 - self.p2)   # rb1 在 rb2 下

        # 2) 解算系旋转角：把 rb3 放到 x 轴
        theta = math.atan2(r32_rb2[1], r32_rb2[0])
        Rs = rot2d(-theta)  # rb2 -> solver
        Rt = rot2d(+theta)  # solver -> rb2

        # 3) 基于里程计的“侧别判据”：看 r12 在解算系下的 y 符号
        r12_s = Rs @ r12_rb2
        desired_branch = self.branch_sign  # 默认维持
        if   r12_s[1] >  self.y_deadzone: desired_branch = +1
        elif r12_s[1] < -self.y_deadzone: desired_branch = -1
        # 迟滞
        if self.branch_sign is None:
            self.branch_sign = desired_branch if desired_branch is not None else +1
        elif desired_branch is not None and desired_branch != self.branch_sign:
            self._flip_count += 1
            if self._flip_count >= self.flip_hysteresis:
                self.branch_sign = desired_branch
                self._flip_count = 0
        else:
            self._flip_count = 0

        # 4) 闭式两解，按 branch_sign 选一支
        p_plus_s, p_minus_s = trilat_candidates_solver_frame(d12, d13, d23)
        cand_s = p_plus_s if self.branch_sign > 0 else p_minus_s

        # 5) 旋回到 rb2/base_link 并发布
        self.pos1_rb2 = Rt @ cand_s
        self.pos3_rb2 = Rt @ np.array([d23, 0.0], float)#r32_rb2
        self.publish_pose1(msg.header.stamp, self.pos1_rb2)
        self.publish_pose3(msg.header.stamp, self.pos3_rb2)
        self.last_process_time = Time.from_msg(msg.header.stamp)

    # ---------- 首帧兜底（已知初始） ----------

    def init_first_frame(self, d12: float, d13: float, d23: float):
        R_w2b0 = rot2d(-math.radians(self.init_rb2_yaw_deg))
        r32_rb2 = R_w2b0 @ (self.init_rb3_xy - self.init_rb2_xy)
        r12_rb2 = R_w2b0 @ (self.init_rb1_xy - self.init_rb2_xy)

        theta = math.atan2(r32_rb2[1], r32_rb2[0])
        Rs = rot2d(-theta); Rt = rot2d(+theta)
        r12_s = Rs @ r12_rb2

        p_plus_s, p_minus_s = trilat_candidates_solver_frame(d12, d13, d23)
        cand_s = p_plus_s if abs(p_plus_s[1]-r12_s[1]) <= abs(p_minus_s[1]-r12_s[1]) else p_minus_s
        self.branch_sign = +1 if cand_s[1] >= 0 else -1

        return Rt @ cand_s

    # ---------- 发布 ----------

    def publish_pose1(self, stamp, pos_rb2_xy: np.ndarray):
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.rb2_frame_id
        out.pose.pose = Pose(
            position=Point(x=float(pos_rb2_xy[0]), y=float(pos_rb2_xy[1]), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        cov = [0.0]*36
        cov[0] = 0.005   # var(x)
        cov[7] = 0.005   # var(y)
        cov[14] = 1e6   # var(z)
        cov[21] = 1e6   # var(roll)
        cov[28] = 1e6   # var(pitch)
        cov[35] = 1e6   # var(yaw)
        out.pose.covariance = cov
        self.pub_pose1.publish(out)

        if self.pub_tf and self.br is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.rb2_frame_id
            tf.child_frame_id = self.rb1_child_frame_id
            tf.transform.translation.x = float(pos_rb2_xy[0])
            tf.transform.translation.y = float(pos_rb2_xy[1])
            tf.transform.translation.z = 0.0
            tf.transform.rotation.w = 1.0
            self.br.sendTransform(tf)

    def publish_pose3(self, stamp, pos_rb2_xy: np.ndarray):
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.rb2_frame_id
        out.pose.pose = Pose(
            position=Point(x=float(pos_rb2_xy[0]), y=float(pos_rb2_xy[1]), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        cov = [0.0]*36
        cov[0] = 0.005   # var(x)
        cov[7] = 0.005   # var(y)
        cov[14] = 1e6   # var(z)
        cov[21] = 1e6   # var(roll)
        cov[28] = 1e6   # var(pitch)
        cov[35] = 1e6   # var(yaw)
        out.pose.covariance = cov
        self.pub_pose3.publish(out)

        if self.pub_tf and self.br is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.rb2_frame_id
            tf.child_frame_id = self.rb3_child_frame_id
            tf.transform.translation.x = float(pos_rb2_xy[0])
            tf.transform.translation.y = float(pos_rb2_xy[1])
            tf.transform.translation.z = 0.0
            tf.transform.rotation.w = 1.0
            self.br.sendTransform(tf)


def main():
    rclpy.init()
    node = TrilatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
