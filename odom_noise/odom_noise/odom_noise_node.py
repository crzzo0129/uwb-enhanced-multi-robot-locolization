#!/usr/bin/env python3
import math, time, random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw*0.5)
    q.z = math.sin(yaw*0.5)
    return q

def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')

        # 参数
        self.in_topic  = self.declare_parameter('in_topic',  '/rb1/odom').value
        self.out_topic = self.declare_parameter('out_topic', '/rb1/odom_noisy').value
        self.seed      = int(self.declare_parameter('seed',  12345).value)

        # 白噪声标准差（测量噪声）
        self.sigma_x   = float(self.declare_parameter('sigma_x',   0.03).value)  # m
        self.sigma_y   = float(self.declare_parameter('sigma_y',   0.03).value)  # m
        self.sigma_yaw = float(self.declare_parameter('sigma_yaw', 0.02).value)  # rad

        # 慢漂（随机游走偏置）强度：每秒标准差
        self.bias_rate_x   = float(self.declare_parameter('bias_rate_x',   0.003).value)  # m/sqrt(s)
        self.bias_rate_y   = float(self.declare_parameter('bias_rate_y',   0.003).value)  # m/sqrt(s)
        self.bias_rate_yaw = float(self.declare_parameter('bias_rate_yaw', 0.003).value)  # rad/sqrt(s)

        # 漂移一阶低通时间常数（秒）
        self.bias_tau = float(self.declare_parameter('bias_tau', 10.0).value)

        # 是否修改 twist
        self.noise_twist = bool(self.declare_parameter('noise_twist', False).value)

        # 协方差放大参数
        self.cov_scale = float(self.declare_parameter('cov_scale', 70.0).value)

        # 初始化随机
        random.seed(self.seed)
        np.random.seed(self.seed)

        # 偏置状态（随机游走累积）
        self.bx = 0.0; self.by = 0.0; self.byaw = 0.0

        # 缓存最新数据（供定时器使用）
        self.latest_msg = None
        self.latest_t = None
        self.last_process_t = None

        # QoS
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # 订阅：只接收数据并更新偏置，不发布
        self.sub = self.create_subscription(Odometry, self.in_topic, self.on_odom, qos)
        self.pub = self.create_publisher(Odometry, self.out_topic, qos)

        # 【关键】独立20Hz定时器（0.05s周期），精确控制发布频率
        self.create_timer(1.0/17.0, self.timer_publish)

        # 统计信息
        self.pub_count = 0
        self.input_count = 0
        self.create_timer(5.0, self.print_stats)  # 每5秒打印一次统计

        self.get_logger().info(
            f"[OdomNoise] in={self.in_topic}, out={self.out_topic}, "
            f"pub_rate=20.0Hz (fixed), σx={self.sigma_x}, σy={self.sigma_y}, σyaw={self.sigma_yaw}, "
            f"bias_rate(x,y,yaw)=({self.bias_rate_x},{self.bias_rate_y},{self.bias_rate_yaw})"
        )

    def on_odom(self, msg: Odometry):
        """
        高频回调（约50Hz）：
        - 更新随机游走偏置（保持连续性）
        - 缓存最新消息和时间戳
        - 不发布！
        """
        self.input_count += 1

        # 时间计算
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        if self.last_process_t is None:
            # 第一帧：初始化
            self.last_process_t = t
            self.latest_t = t
            self.latest_msg = msg
            return

        # 计算dt（时间步长）
        dt = max(1e-6, t - self.last_process_t)
        self.last_process_t = t
        self.latest_t = t
        self.latest_msg = msg

        # 【关键】每帧都更新偏置（随机游走需要连续积分）
        # 即使不定频发布，偏置也要保持50Hz更新，确保漂移连续性
        alpha = 0.0 if self.bias_tau <= 1e-3 else (1.0 - dt/self.bias_tau)

        # 一阶低通 + 白噪声驱动
        self.bx   = alpha*self.bx   + np.random.normal(0.0, self.bias_rate_x*math.sqrt(dt))
        self.by   = alpha*self.by   + np.random.normal(0.0, self.bias_rate_y*math.sqrt(dt))
        self.byaw = alpha*self.byaw + np.random.normal(0.0, self.bias_rate_yaw*math.sqrt(dt))

        # 注意：这里不发布！只更新状态

    def timer_publish(self):
        """
        独立定时器回调（严格20Hz，0.05s周期）：
        - 使用缓存的最新消息
        - 叠加当前的累积偏置 + 瞬时白噪声
        - 发布带噪声的odom
        """
        if self.latest_msg is None:
            return

        msg = self.latest_msg
        t = self.latest_t

        # 读原始位姿
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        # 注入白噪声（仅在发布时采样，20Hz）
        nx   = np.random.normal(0.0, self.sigma_x)
        ny   = np.random.normal(0.0, self.sigma_y)
        nyaw = np.random.normal(0.0, self.sigma_yaw)

        # 最终位姿 = 真值 + 累积偏置（50Hz连续更新）+ 白噪声（20Hz采样）
        x_n   = x   + self.bx   + nx
        y_n   = y   + self.by   + ny
        yaw_n = wrap(yaw + self.byaw + nyaw)

        # 构造输出消息
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.header.frame_id = msg.header.frame_id

        out.pose.pose.position.x = float(x_n)
        out.pose.pose.position.y = float(y_n)
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation = quat_from_yaw(yaw_n)

        # 更新协方差（保守估计，加入偏置等效项）
        C = np.array(msg.pose.covariance).reshape(6,6)
        # 假设发布周期约0.05s
        dt_pub = 1.0/17.0
        C[0,0] += self.sigma_x**2 + (self.bias_rate_x**2)*dt_pub
        C[1,1] += self.sigma_y**2 + (self.bias_rate_y**2)*dt_pub
        C[5,5] += self.sigma_yaw**2 + (self.bias_rate_yaw**2)*dt_pub

        C[0,0] *= self.cov_scale
        C[1,1] *= self.cov_scale
        C[5,5] *= self.cov_scale
        out.pose.covariance = list(C.reshape(-1))

        # twist原样传递（或按需加噪）
        out.twist = msg.twist

        self.pub.publish(out)
        self.pub_count += 1

    def print_stats(self):
        """打印频率统计信息"""
        if self.input_count > 0:
            actual_rate = self.pub_count / 5.0  # 过去5秒的平均发布频率
            self.get_logger().info(
                f"[Stats] Input: {self.input_count/5.0:.1f}Hz, "
                f"Output: {actual_rate:.1f}Hz (target: 20.0Hz), "
                f"Bias: ({self.bx:.4f}, {self.by:.4f}, {self.byaw:.4f})"
            )
            self.input_count = 0
            self.pub_count = 0

def main():
    rclpy.init()
    rclpy.spin(OdomNoise())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
