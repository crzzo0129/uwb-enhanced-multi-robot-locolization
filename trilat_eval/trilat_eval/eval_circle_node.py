# trilat_eval/eval_circle_node.py
import math
from collections import deque
from typing import Deque, Tuple, List
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

import matplotlib
matplotlib.use('Agg')  # 后端设为无界面
import matplotlib.pyplot as plt

def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def circle_fit_kasa(x: np.ndarray, y: np.ndarray):
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    b = x**2 + y**2
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = c[0], c[1]
    R = math.sqrt(max(1e-12, c[2] + cx*cx + cy*cy))
    return cx, cy, R

def curvature_three_points(p0, p1, p2):
    x0,y0 = p0; x1,y1 = p1; x2,y2 = p2
    a = math.hypot(x1-x0, y1-y0)
    b = math.hypot(x2-x1, y2-y1)
    c = math.hypot(x2-x0, y2-y0)
    if a < 1e-9 or b < 1e-9 or c < 1e-9: return 0.0
    area2 = abs((x1-x0)*(y2-y0) - (y1-y0)*(x2-x0))
    return area2 / (a*b*c)

def nearest_sample(buf: Deque[Tuple[float,float,float,float]], t: float):
    if not buf: return None
    return min(buf, key=lambda s: abs(s[0]-t))

class EvalCircleNode(Node):
    def __init__(self):
        super().__init__('eval_circle_node')
        # topics
        self.robot_name = self.declare_parameter('robot_name', 'rb1').value
        self.gt_entity  = self.declare_parameter('gt_entity', 'robot1').value
        self.gt_topic   = self.declare_parameter('gt_topic', '/model_states').value
        self.odom_topic = self.declare_parameter('odom_topic', f'/{self.robot_name}/odom').value
        self.filt_topic = self.declare_parameter('filt_topic', f'/{self.robot_name}/odometry/filtered').value
        # eval
        self.window_sec = float(self.declare_parameter('window_sec', 60.0).value)
        self.report_period = float(self.declare_parameter('report_period', 5.0).value)
        self.min_points = int(self.declare_parameter('min_points', 200).value)
        self.save_csv = bool(self.declare_parameter('save_csv', False).value)
        self.csv_path = self.declare_parameter('csv_path', '/tmp/trilat_eval.csv').value
        # figure
        self.save_fig = bool(self.declare_parameter('save_fig', True).value)
        self.fig_path = self.declare_parameter('fig_path', '/tmp/trilat_eval.png').value
        self.fig_size = tuple(self.declare_parameter('fig_size', [6.0, 6.0]).value)

        # buffers: (t, x, y, yaw)
        self.gt_buf, self.odom_buf, self.filt_buf = deque(), deque(), deque()

        self.create_subscription(ModelStates, self.gt_topic, self.on_gt, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(Odometry, self.filt_topic, self.on_filt, 50)
        self.timer = self.create_timer(self.report_period, self.on_report)

        self.get_logger().info(f"[eval] robot={self.robot_name}, entity={self.gt_entity}")

    # ---- callbacks ----
    def on_gt(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.gt_entity)
        except ValueError:
            return
        p = msg.pose[idx].position; q = msg.pose[idx].orientation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        t = float(self.get_clock().now().nanoseconds) * 1e-9
        self.gt_buf.append((t, p.x, p.y, yaw)); self.trim(self.gt_buf, t)

    def on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_buf.append((t, p.x, p.y, yaw)); self.trim(self.odom_buf, t)

    def on_filt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        self.filt_buf.append((t, p.x, p.y, yaw)); self.trim(self.filt_buf, t)

    # ---- utils ----
    def trim(self, buf: Deque, now_t: float):
        win = self.window_sec
        while buf and (now_t - buf[0][0] > win):
            buf.popleft()

    @staticmethod
    def quat_to_yaw(x,y,z,w) -> float:
        siny_cosp = 2.0 * (w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---- periodic report & figure ----
    def on_report(self):
        if len(self.gt_buf) < self.min_points or len(self.odom_buf) < self.min_points or len(self.filt_buf) < self.min_points:
            self.get_logger().info("collecting…")
            return

        # 1) fit circle from GT
        gt_xy = np.array([(x,y) for (_,x,y,_) in self.gt_buf], dtype=float)
        cx, cy, R = circle_fit_kasa(gt_xy[:,0], gt_xy[:,1])

        # 2) align to filtered timestamps
        t_list = [t for (t,_,_,_) in self.filt_buf]
        gt_samples   = np.array([nearest_sample(self.gt_buf,  t) for t in t_list], dtype=object)
        odom_samples = np.array([nearest_sample(self.odom_buf,t) for t in t_list], dtype=object)
        filts        = np.array(self.filt_buf, dtype=object)

        def extract_xy(samples):
            xs = np.array([s[1] for s in samples], float)
            ys = np.array([s[2] for s in samples], float)
            return xs, ys

        gx, gy = extract_xy(gt_samples)
        ox, oy = extract_xy(odom_samples)
        fx, fy = extract_xy(filts)

        # 3) metrics（与先前一致，这里省略打印细节以专注绘图）
        # ... 你可复用之前版本里的统计 ...

        # 4) draw figure: ideal circle (red), odom points (yellow), filt points (green)
        if self.save_fig:
            fig = plt.figure(figsize=self.fig_size, dpi=150)
            ax = fig.add_subplot(111)
            # ideal circle
            th = np.linspace(0, 2*np.pi, 512)
            ax.plot(cx + R*np.cos(th), cy + R*np.sin(th), '-', linewidth=1.5, color='red', label='ideal circle')
            # odom & filt points
            ax.scatter(ox, oy, s=8, color='blue', edgecolors='none', label='odom')
            ax.scatter(fx, fy, s=8, color='green',  edgecolors='none', label='filtered')
            ax.set_aspect('equal', 'box')
            # 可选：关闭坐标轴刻度，让图更干净
            #ax.set_xticks([]); ax.set_yticks([])
            ax.legend(loc='upper right', fontsize=8, frameon=False)
            fig.tight_layout()
            try:
                fig.savefig(self.fig_path)
                self.get_logger().info(f"[eval] figure saved: {self.fig_path}")
            except Exception as e:
                self.get_logger().warn(f"save fig failed: {e}")
            plt.close(fig)

def main():
    rclpy.init()
    node = EvalCircleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()