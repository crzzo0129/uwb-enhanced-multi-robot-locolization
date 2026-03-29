#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import csv
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path

# 可选 Gazebo 支持
try:
    from gazebo_msgs.msg import ModelStates
    HAS_GAZEBO = True
except ImportError:
    HAS_GAZEBO = False

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def yaw_from_quat(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def cumulative_arc_length(xs, ys):
    dx = np.diff(xs)
    dy = np.diff(ys)
    seg = np.hypot(dx, dy)
    S = np.zeros_like(xs)
    S[1:] = np.cumsum(seg)
    return S


def project_point_to_polyline(qx, qy, xs, ys):
    """简化版投影，只返回 signed cross-track error"""
    best_dist = float('inf')
    best_signed_xt = 0.0
    for j in range(len(xs)-1):
        vx = xs[j+1] - xs[j]
        vy = ys[j+1] - ys[j]
        wx = qx - xs[j]
        wy = qy - ys[j]
        seg_len2 = vx*vx + vy*vy
        if seg_len2 < 1e-12:
            d = math.hypot(wx, wy)
            if d < best_dist:
                best_dist = d
                best_signed_xt = 0.0
            continue
        u = max(0.0, min(1.0, (wx*vx + wy*vy) / seg_len2))
        hx = xs[j] + u * vx
        hy = ys[j] + u * vy
        px, py = qx - hx, qy - hy
        d = math.hypot(px, py)
        cross = vx * py - vy * px
        signed_xt = math.copysign(d, cross)
        if d < best_dist:
            best_dist = d
            best_signed_xt = signed_xt
    return best_signed_xt


class EvalPolylineNode(Node):
    def __init__(self):
        super().__init__('eval_polyline_node')

        # 参数
        self.declare_parameter('gt_source', 'gazebo')
        self.declare_parameter('gazebo_model_name', 'robot1')
        self.declare_parameter('gt_path_topic', '/gt_path')
        self.declare_parameter('odom_topic', '/rb1/odom_noisy')
        self.declare_parameter('filtered_topic', '/rb1/ukf_pose')
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('save_fig', True)
        self.declare_parameter('fig_path', '/home/rhw/fusion_ws/src/trilat_eval/results/eval_rb1.png')
        self.declare_parameter('write_csv', True)
        self.declare_parameter('csv_path', '/home/rhw/fusion_ws/src/trilat_eval/results/eval_rb1.csv')

        self.gt_source = self.get_parameter('gt_source').value
        self.gazebo_model_name = self.get_parameter('gazebo_model_name').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.filtered_topic = self.get_parameter('filtered_topic').value
        self.report_period = self.get_parameter('report_period').value
        self.save_fig = self.get_parameter('save_fig').value
        self.fig_path = self.get_parameter('fig_path').value
        self.write_csv = self.get_parameter('write_csv').value
        self.csv_path = self.get_parameter('csv_path').value

        # 数据缓冲
        self.gt_buf: List[Tuple[float, float, float]] = []      # (t, x, y)
        self.odom_buf: List[Tuple[float, float, float]] = []    # (t, x, y)
        self.filt_buf: List[Tuple[float, float, float]] = []    # (t, x, y)

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=100)

        # 订阅
        if self.gt_source == 'gazebo' and HAS_GAZEBO:
            self.create_subscription(ModelStates, '/model_states', self.on_model_states, qos)
        else:
            self.create_subscription(Path, self.get_parameter('gt_path_topic').value, self.on_gt_path, qos)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)
        self.create_subscription(PoseWithCovarianceStamped, self.filtered_topic, self.on_filtered, qos)

        self.create_timer(self.report_period, self.on_report)

        self.get_logger().info(f"Eval node started | odom={self.odom_topic} | filtered={self.filtered_topic}")

    def on_model_states(self, msg):
        try:
            idx = msg.name.index(self.gazebo_model_name)
            p = msg.pose[idx].position
            t = self.get_clock().now().nanoseconds * 1e-9
            self.gt_buf.append((t, float(p.x), float(p.y)))
        except ValueError:
            pass

    def on_gt_path(self, msg):
        for pose in msg.poses:
            t = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
            p = pose.pose.position
            self.gt_buf.append((t, float(p.x), float(p.y)))

    def on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.odom_buf.append((t, x, y))
        if len(self.odom_buf) > 15000:
            self.odom_buf = self.odom_buf[-15000:]

    def on_filtered(self, msg: PoseWithCovarianceStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.filt_buf.append((t, x, y))
        if len(self.filt_buf) > 15000:
            self.filt_buf = self.filt_buf[-15000:]

    def on_report(self):
        if len(self.gt_buf) < 50 or len(self.odom_buf) < 50 or len(self.filt_buf) < 30:
            self.get_logger().info(f"Collecting... gt={len(self.gt_buf)} odom={len(self.odom_buf)} filt={len(self.filt_buf)}")
            return

        # 转为 numpy
        gt_np = np.array(self.gt_buf)
        gx, gy = gt_np[:, 1], gt_np[:, 2]

        fi_np = np.array(self.filt_buf)
        fi_x, fi_y = fi_np[:, 1], fi_np[:, 2]

        od_np = np.array(self.odom_buf)
        od_x, od_y = od_np[:, 1], od_np[:, 2]

        # 简单投影（cross-track error）
        xt_odom = np.array([project_point_to_polyline(x, y, gx, gy) for x, y in zip(od_x, od_y)])
        xt_filt = np.array([project_point_to_polyline(x, y, gx, gy) for x, y in zip(fi_x, fi_y)])

        metrics = {
            'odom': {'xt_rmse': float(np.sqrt(np.mean(xt_odom**2))), 'xt_p95': float(np.percentile(np.abs(xt_odom), 95))},
            'filtered': {'xt_rmse': float(np.sqrt(np.mean(xt_filt**2))), 'xt_p95': float(np.percentile(np.abs(xt_filt), 95))}
        }

        self.get_logger().info(f"[EVAL] Odom XT_RMSE: {metrics['odom']['xt_rmse']:.4f} m | "
                               f"Filtered XT_RMSE: {metrics['filtered']['xt_rmse']:.4f} m")

        # 保存 CSV
        if self.write_csv:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['t', 'odom_x', 'odom_y', 'filt_x', 'filt_y', 'xt_odom', 'xt_filt'])
                for i in range(len(fi_np)):
                    writer.writerow([
                        fi_np[i, 0], od_x[i], od_y[i], fi_x[i], fi_y[i],
                        xt_odom[i], xt_filt[i]
                    ])
            self.get_logger().info(f"CSV saved: {self.csv_path}")

        # 保存图片
        if self.save_fig:
            plt.figure(figsize=(10, 8))
            plt.plot(gx, gy, 'k-', linewidth=2, label='GT Polyline')
            plt.scatter(od_x, od_y, s=20, c='red', alpha=0.6, label='odom_noisy')
            plt.scatter(fi_x, fi_y, s=15, c='blue', alpha=0.9, label='UKF (Range-only)')
            plt.axis('equal')
            plt.grid(True, linestyle='--', alpha=0.5)
            plt.legend()
            plt.title('Localization Evaluation')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.savefig(self.fig_path, dpi=200, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f"Figure saved: {self.fig_path}")


def main():
    rclpy.init()
    node = EvalPolylineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()