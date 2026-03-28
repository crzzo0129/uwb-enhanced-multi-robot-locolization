#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Eval arbitrary-trajectory localization accuracy by projecting odom/filtered points
onto a GT polyline. Replaces circle-fitting evaluation with general polyline projection.

ROS 2 (rclpy) node.

Author: ChatGPT
"""

import math
import os
import csv
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

# Gazebo model states are optional. If not present, user can set gt_source=path and feed a nav_msgs/Path.
try:
    from gazebo_msgs.msg import ModelStates  # type: ignore
    HAS_GAZEBO = True
except Exception:
    HAS_GAZEBO = False

# Matplotlib is used only for saving figures offline.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def yaw_from_quat(x, y, z, w) -> float:
    """Compute yaw from quaternion (z,w assumed significant for planar motion)."""
    # Standard yaw extraction
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def cumulative_arc_length(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    dx = np.diff(xs); dy = np.diff(ys)
    seg = np.hypot(dx, dy)
    S = np.zeros_like(xs)
    S[1:] = np.cumsum(seg)
    return S  # same length as xs/ys


def project_point_to_polyline(qx: float, qy: float, xs: np.ndarray, ys: np.ndarray):
    """
    Project point q=(qx,qy) to a polyline described by xs, ys.
    Returns: (dist, j, u, hx, hy, s_proj, signed_xt)
      j: index of segment start
      u in [0,1]: param on segment
      (hx,hy): projection
      s_proj: arc length at projection
      signed_xt: signed cross-track error (right-hand rule w.r.t. segment direction)
    """
    dx = np.diff(xs); dy = np.diff(ys)
    segL2 = dx*dx + dy*dy
    best = (float('inf'), -1, 0.0, 0.0, 0.0, 0.0, 0.0)
    S = cumulative_arc_length(xs, ys)
    for j in range(len(xs)-1):
        vx, vy = dx[j], dy[j]
        wx, wy = qx - xs[j], qy - ys[j]
        if segL2[j] < 1e-12:
            d = math.hypot(wx, wy)
            if d < best[0]:
                best = (d, j, 0.0, xs[j], ys[j], S[j], 0.0)
            continue
        u = (wx*vx + wy*vy) / segL2[j]
        u = 0.0 if u < 0.0 else 1.0 if u > 1.0 else u
        hx, hy = xs[j] + u*vx, ys[j] + u*vy
        px, py = qx - hx, qy - hy
        d = math.hypot(px, py)
        # signed cross-track using cross product sign
        cross = vx*(qy - ys[j]) - vy*(qx - xs[j])
        signed_xt = math.copysign(d, cross)
        s_proj = S[j] + u*math.hypot(vx, vy)
        if d < best[0]:
            best = (d, j, u, hx, hy, s_proj, signed_xt)
    return best


def rmse(a: np.ndarray) -> float:
    return float(np.sqrt(np.mean(a*a))) if a.size > 0 else float('nan')


def p95(a: np.ndarray) -> float:
    return float(np.percentile(np.abs(a), 95)) if a.size > 0 else float('nan')


class EvalPolylineNode(Node):
    """
    Evaluate odom and filtered odometry against a GT polyline.
    GT can come from Gazebo ModelStates (by model_name) or from a nav_msgs/Path topic.
    """

    def __init__(self):
        super().__init__('eval_polyline_node')

        # Parameters
        self.declare_parameter('gt_source', 'gazebo')  # 'gazebo' or 'path'
        self.declare_parameter('gazebo_model_name', 'robot')
        self.declare_parameter('gt_path_topic', '/gt_path')  # used when gt_source == 'path'
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('filtered_topic', '/odometry/filtered')
        self.declare_parameter('min_points', 50)
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('save_fig', True)
        self.declare_parameter('fig_path', 'eval_polyline.png')
        self.declare_parameter('fig_size', [6.0, 6.0])
        self.declare_parameter('write_csv', True)
        self.declare_parameter('csv_path', 'eval_polyline_metrics.csv')

        self.gt_source: str = self.get_parameter('gt_source').get_parameter_value().string_value
        self.gazebo_model_name: str = self.get_parameter('gazebo_model_name').get_parameter_value().string_value
        self.gt_path_topic: str = self.get_parameter('gt_path_topic').get_parameter_value().string_value
        self.odom_topic: str = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.filtered_topic: str = self.get_parameter('filtered_topic').get_parameter_value().string_value
        self.min_points: int = self.get_parameter('min_points').get_parameter_value().integer_value
        self.report_period: float = self.get_parameter('report_period').get_parameter_value().double_value
        self.save_fig: bool = self.get_parameter('save_fig').get_parameter_value().bool_value
        self.fig_path: str = self.get_parameter('fig_path').get_parameter_value().string_value
        self.fig_size: List[float] = [float(x) for x in self.get_parameter('fig_size').get_parameter_value().double_array_value]
        self.write_csv: bool = self.get_parameter('write_csv').get_parameter_value().bool_value
        self.csv_path: str = self.get_parameter('csv_path').get_parameter_value().string_value

        # Buffers: list of tuples (t, x, y, yaw)
        self.gt_buf: List[Tuple[float, float, float, float]] = []
        self.odom_buf: List[Tuple[float, float, float, float]] = []
        self.filt_buf: List[Tuple[float, float, float, float]] = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )

        # Subscriptions
        if self.gt_source == 'gazebo':
            if not HAS_GAZEBO:
                self.get_logger().error('gt_source set to "gazebo" but gazebo_msgs/ModelStates not available. '
                                        'Switch to gt_source="path" or install gazebo_msgs.')
            else:
                self.create_subscription(ModelStates, '/model_states', self.on_model_states, qos)
        elif self.gt_source == 'path':
            self.create_subscription(Path, self.gt_path_topic, self.on_gt_path, qos)
        else:
            self.get_logger().warn('Unknown gt_source. Use "gazebo" or "path". Fallback to "path".')
            self.create_subscription(Path, self.gt_path_topic, self.on_gt_path, qos)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)
        self.create_subscription(Odometry, self.filtered_topic, self.on_filtered, qos)

        # Periodic report
        self.create_timer(self.report_period, self.on_report)

        self.get_logger().info(f'Initialized eval_polyline_node with gt_source={self.gt_source}, '
                               f'odom="{self.odom_topic}", filtered="{self.filtered_topic}"')

    # ------------------------- Callbacks -------------------------

    def on_model_states(self, msg: 'ModelStates'):
        """Extract GT pose for the target model from Gazebo ModelStates."""
        try:
            idx = msg.name.index(self.gazebo_model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        t = self.get_clock().now().nanoseconds * 1e-9
        x = float(pose.position.x)
        y = float(pose.position.y)
        q = pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.gt_buf.append((t, x, y, yaw))
        # Optional trimming
        if len(self.gt_buf) > 20000:
            self.gt_buf = self.gt_buf[-20000:]

    def on_gt_path(self, msg: Path):
        """Use nav_msgs/Path as GT polyline (timestamps from header.stamp of poses)."""
        for ps in msg.poses:
            t = ps.header.stamp.sec + ps.header.stamp.nanosec * 1e-9
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            q = ps.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            self.gt_buf.append((t, x, y, yaw))
        if len(self.gt_buf) > 20000:
            self.gt_buf = self.gt_buf[-20000:]

    def on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.odom_buf.append((t, x, y, yaw))
        if len(self.odom_buf) > 20000:
            self.odom_buf = self.odom_buf[-20000:]

    def on_filtered(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, y=q.y, z=q.z, w=q.w)
        self.filt_buf.append((t, x, y, yaw))
        if len(self.filt_buf) > 20000:
            self.filt_buf = self.filt_buf[-20000:]

    # ------------------------- Utilities -------------------------

    @staticmethod
    def nearest_sample(buf: List[Tuple[float, float, float, float]], t: float) -> Tuple[float, float, float, float]:
        # Linear search is fine for small windows. For large buffers consider bisect.
        if not buf:
            return (t, float('nan'), float('nan'), float('nan'))
        # Find nearest by absolute time difference
        times = np.array([b[0] for b in buf], dtype=float)
        idx = int(np.argmin(np.abs(times - t)))
        return buf[idx]

    # ------------------------- Reporting -------------------------

    def on_report(self):
        if len(self.gt_buf) < self.min_points or len(self.odom_buf) < self.min_points or len(self.filt_buf) < self.min_points:
            self.get_logger().info('collecting… gt=%d odom=%d filt=%d' % (len(self.gt_buf), len(self.odom_buf), len(self.filt_buf)))
            return

        # A) Build GT polyline sorted by time
        gt_np = np.array(self.gt_buf, dtype=object)
        order = np.argsort(gt_np[:, 0].astype(float))
        gt_np = gt_np[order]
        gx = gt_np[:, 1].astype(float)
        gy = gt_np[:, 2].astype(float)
        gt_t_all = gt_np[:, 0].astype(float)
        gS = cumulative_arc_length(gx, gy)

        # B) Choose evaluation instants from filtered timestamps
        fi_ts = np.array([t for (t, _, _, _) in self.filt_buf], dtype=float)

        # Prepare aligned odom/filtered series (nearest in time)
        od_aligned = np.array([self.nearest_sample(self.odom_buf, t) for t in fi_ts], dtype=object)
        fi_aligned = np.array([self.nearest_sample(self.filt_buf, t) for t in fi_ts], dtype=object)

        od_x = od_aligned[:, 1].astype(float); od_y = od_aligned[:, 2].astype(float)
        fi_x = fi_aligned[:, 1].astype(float); fi_y = fi_aligned[:, 2].astype(float)

        # C) Time-aligned GT arc length reference using nearest GT timestamp indices
        gt_time_idx = np.searchsorted(gt_t_all, fi_ts, side='left')
        gt_time_idx = np.clip(gt_time_idx, 0, len(gt_t_all) - 1)
        s_time_ref = gS[gt_time_idx]

        # D) Spatial projection for odom/filtered
        def batch_project(xs, ys, GX, GY):
            xt_list, sproj_list = [], []
            for qx, qy in zip(xs, ys):
                _, _, _, _, _, s_proj, signed_xt = project_point_to_polyline(qx, qy, GX, GY)
                xt_list.append(signed_xt)
                sproj_list.append(s_proj)
            return np.array(xt_list), np.array(sproj_list)

        xt_odom, sproj_odom = batch_project(od_x, od_y, gx, gy)
        xt_filt, sproj_filt = batch_project(fi_x, fi_y, gx, gy)

        # E) Along-track error under time alignment
        along_odom = sproj_odom - s_time_ref
        along_filt = sproj_filt - s_time_ref

        # F) Metrics
        metrics = {
            'odom': {
                'xt_rmse': rmse(xt_odom),
                'xt_p95':  p95(xt_odom),
                'along_rmse': rmse(along_odom),
            },
            'filtered': {
                'xt_rmse': rmse(xt_filt),
                'xt_p95':  p95(xt_filt),
                'along_rmse': rmse(along_filt),
            }
        }
        self.get_logger().info(f'[eval] metrics: {metrics}')

        # G) Optional CSV dump
        if self.write_csv:
            try:
                header = ['t', 'odom_x', 'odom_y', 'filt_x', 'filt_y', 's_time_ref', 'sproj_odom', 'sproj_filt', 'xt_odom', 'xt_filt', 'along_odom', 'along_filt']
                with open(self.csv_path, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(header)
                    for i in range(len(fi_ts)):
                        row = [
                            float(fi_ts[i]),
                            float(od_x[i]), float(od_y[i]),
                            float(fi_x[i]), float(fi_y[i]),
                            float(s_time_ref[i]),
                            float(sproj_odom[i]), float(sproj_filt[i]),
                            float(xt_odom[i]), float(xt_filt[i]),
                            float(along_odom[i]), float(along_filt[i]),
                        ]
                        w.writerow(row)
                self.get_logger().info(f'[eval] CSV saved: {self.csv_path}')
            except Exception as e:
                self.get_logger().warn(f'CSV save failed: {e}')

        # H) Optional figure
        if self.save_fig:
            try:
                fig = plt.figure(figsize=(self.fig_size[0], self.fig_size[1]), dpi=160)
                ax = fig.add_subplot(111)
                ax.plot(gx, gy, '-', linewidth=1.5, label='GT polyline')
                ax.scatter(od_x, od_y, s=6, label='odom', edgecolors='none')
                ax.scatter(fi_x, fi_y, s=6, label='filtered', edgecolors='none')
                ax.set_aspect('equal', 'box')
                ax.grid(True, linestyle=':', linewidth=0.5)
                ax.legend(loc='best', fontsize=8, frameon=False)
                fig.tight_layout()
                fig.savefig(self.fig_path)
                plt.close(fig)
                self.get_logger().info(f'[eval] Figure saved: {self.fig_path}')
            except Exception as e:
                self.get_logger().warn(f'Figure save failed: {e}')


def main(args=None):
    rclpy.init(args=args)
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
