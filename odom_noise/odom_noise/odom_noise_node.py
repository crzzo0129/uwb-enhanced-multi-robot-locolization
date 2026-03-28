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

        # 漂移一阶低通时间常数（秒），让偏置更平滑
        self.bias_tau = float(self.declare_parameter('bias_tau', 10.0).value)

        # 是否修改 twist
        self.noise_twist = bool(self.declare_parameter('noise_twist', False).value)

        #协方差放大参数
        self.cov_scale = float(self.declare_parameter('cov_scale', 70.0).value)

        # 初始化随机
        random.seed(self.seed)
        np.random.seed(self.seed)

        # 偏置与时间
        self.bx = 0.0; self.by = 0.0; self.byaw = 0.0
        self.last_t = None

        # QoS 与通信
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(Odometry, self.in_topic, self.on_odom, qos)
        self.pub = self.create_publisher(Odometry, self.out_topic, qos)

        self.get_logger().info(
            f"in={self.in_topic}, out={self.out_topic}, "
            f"σx={self.sigma_x}, σy={self.sigma_y}, σyaw={self.sigma_yaw}, "
            f"bias_rate(x,y,yaw)=({self.bias_rate_x},{self.bias_rate_y},{self.bias_rate_yaw}), "
            f"tau={self.bias_tau}, noise_twist={self.noise_twist}"
        )

    def on_odom(self, msg: Odometry):
        # 时间步长（用消息时戳，优先 sim_time）
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.last_t is None:
            self.last_t = t
            # 直接转发第一帧（或也可加一次白噪声）
            out = msg
            self.pub.publish(out)
            return
        dt = max(1e-3, t - self.last_t)
        self.last_t = t

        # 随机游走偏置更新：一阶低通 + 白噪声驱动
        # b_{k+1} = (1 - dt/tau)*b_k + N(0, (bias_rate^2 * dt))
        alpha = 0.0 if self.bias_tau <= 1e-3 else (1.0 - dt/self.bias_tau)
        self.bx   = alpha*self.bx   + np.random.normal(0.0, self.bias_rate_x*math.sqrt(dt))
        self.by   = alpha*self.by   + np.random.normal(0.0, self.bias_rate_y*math.sqrt(dt))
        self.byaw = alpha*self.byaw + np.random.normal(0.0, self.bias_rate_yaw*math.sqrt(dt))

        # 读原始
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        # 注入白噪声
        nx   = np.random.normal(0.0, self.sigma_x)
        ny   = np.random.normal(0.0, self.sigma_y)
        nyaw = np.random.normal(0.0, self.sigma_yaw)

        # 新的测量
        x_n   = x   + self.bx   + nx
        y_n   = y   + self.by   + ny
        yaw_n = wrap(yaw + self.byaw + nyaw)

        # 构造输出消息
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.header.frame_id = msg.header.frame_id  # 一般是 odom

        out.pose.pose.position.x = float(x_n)
        out.pose.pose.position.y = float(y_n)
        out.pose.pose.position.z = msg.pose.pose.position.z
        out.pose.pose.orientation = quat_from_yaw(yaw_n)

        # 更新 pose 协方差：在原有基础上加噪声方差 + 偏置等效项（保守地加 dt 量级）
        C = np.array(msg.pose.covariance).reshape(6,6)
        C[0,0] += self.sigma_x**2 + (self.bias_rate_x**2)*dt
        C[1,1] += self.sigma_y**2 + (self.bias_rate_y**2)*dt
        C[5,5] += self.sigma_yaw**2 + (self.bias_rate_yaw**2)*dt

        # 直接放缩
        C[0,0] *= self.cov_scale
        C[1,1] *= self.cov_scale
        C[5,5] *= self.cov_scale
        out.pose.covariance = list(C.reshape(-1))

        # twist 是否加噪
        if self.noise_twist:
            out.twist = msg.twist
            # 可按需对 linear.x / angular.z 再加白噪声或慢漂
        else:
            out.twist = msg.twist  # 原样传递

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(OdomNoise())
    rclpy.shutdown()
