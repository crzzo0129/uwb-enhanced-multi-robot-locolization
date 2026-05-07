#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import message_filters  # 需要添加时间同步

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


def project_point_to_polyline(qx, qy, xs, ys):
    """返回 signed cross-track error"""
    best_dist = float('inf')
    best_signed_xt = 0.0
    for j in range(len(xs)-1):
        vx = xs[j+1] - xs[j]
        vy = ys[j+1] - ys[j]
        wx = qx - xs[j]
        wy = qy - ys[j]
        seg_len2 = vx*vx + vy*vy
        if seg_len2 < 1e-12:
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
        self.declare_parameter('gt_source', 'gazebo')  # 'gazebo' 或 'path'
        self.declare_parameter('gazebo_model_name', 'robot1')
        self.declare_parameter('odom_noisy_topic', '/rb1/odom_noisy')
        self.declare_parameter('filtered_topic', '/rb1/ukf_pose')
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('save_fig', True)
        self.declare_parameter('fig_path', '/home/rhw/fusion_ws/src/trilat_eval/results/eval_rb1.png')
        self.declare_parameter('write_csv', True)
        self.declare_parameter('csv_path', '/home/rhw/fusion_ws/src/trilat_eval/results/eval_rb1.csv')

        # 同步缓冲区 - 使用字典按时间戳存储，确保对齐
        self.synced_data = []  # 每项: {t, gt_x, gt_y, noisy_x, noisy_y, filt_x, filt_y}

        # 临时缓冲区（用于插值对齐）
        self.gt_buffer = []      # [(t, x, y), ...]
        self.noisy_buffer = []   # [(t, x, y), ...]
        self.filt_buffer = []    # [(t, x, y), ...]

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=100
        )

        # 订阅真值（Gazebo）
        if self.get_parameter('gt_source').value == 'gazebo':
            self.create_subscription(
                ModelStates, '/gazebo/model_states', 
                self.on_model_states, qos
            )
        else:
            self.create_subscription(
                Path, '/gt_path', self.on_gt_path, qos
            )

        # 使用 ApproximateTimeSynchronizer 同步 noisy 和 filtered（关键优化！）
        noisy_sub = message_filters.Subscriber(
            self, Odometry, 
            self.get_parameter('odom_noisy_topic').value,
            qos_profile=qos
        )
        filt_sub = message_filters.Subscriber(
            self, PoseWithCovarianceStamped,
            self.get_parameter('filtered_topic').value,
            qos_profile=qos
        )
        
        # 同步器：允许 100ms 的时间差
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [noisy_sub, filt_sub], 
            queue_size=100, 
            slop=0.1
        )
        self.sync.registerCallback(self.on_synced_data)

        # 定时报告
        self.create_timer(self.get_parameter('report_period').value, self.on_report)

        self.get_logger().info(
            f"Eval started | GT=Gazebo({self.get_parameter('gazebo_model_name').value}) | "
            f"Noisy={self.get_parameter('odom_noisy_topic').value} | "
            f"Filtered={self.get_parameter('filtered_topic').value}"
        )

    def on_model_states(self, msg):
        """接收Gazebo真值"""
        try:
            idx = msg.name.index(self.get_parameter('gazebo_model_name').value)
            p = msg.pose[idx].position
            t = self.get_clock().now().nanoseconds * 1e-9
            self.gt_buffer.append((t, float(p.x), float(p.y)))
            # 限制缓冲区大小
            if len(self.gt_buffer) > 1000:
                self.gt_buffer.pop(0)
        except ValueError:
            pass

    def on_gt_path(self, msg):
        """接收Path格式的真值"""
        for pose in msg.poses:
            t = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
            p = pose.pose.position
            self.gt_buffer.append((t, float(p.x), float(p.y)))
        if len(self.gt_buffer) > 1000:
            self.gt_buffer = self.gt_buffer[-1000:]

    def on_synced_data(self, noisy_msg, filt_msg):
        """
        同步回调：noisy_odom 和 ukf_pose 时间对齐后触发
        然后查找最接近的真值
        """
        # 提取时间戳
        t_noisy = noisy_msg.header.stamp.sec + noisy_msg.header.stamp.nanosec * 1e-9
        t_filt = filt_msg.header.stamp.sec + filt_msg.header.stamp.nanosec * 1e-9
        t = (t_noisy + t_filt) / 2  # 使用平均时间
        
        # 提取数据
        noisy_x = float(noisy_msg.pose.pose.position.x)
        noisy_y = float(noisy_msg.pose.pose.position.y)
        filt_x = float(filt_msg.pose.pose.position.x)
        filt_y = float(filt_msg.pose.pose.position.y)
        
        # 查找最接近的真值（时间差 < 50ms）
        gt_x, gt_y = self.find_nearest_gt(t)
        
        if gt_x is not None:
            self.synced_data.append({
                't': t,
                'gt_x': gt_x,
                'gt_y': gt_y,
                'noisy_x': noisy_x,
                'noisy_y': noisy_y,
                'filt_x': filt_x,
                'filt_y': filt_y
            })
            
            # 限制总数据量
            if len(self.synced_data) > 5000:
                self.synced_data.pop(0)

    def find_nearest_gt(self, t_query):
        """在gt_buffer中找到时间最接近的真值"""
        if not self.gt_buffer:
            return None, None
            
        # 简单线性搜索（buffer不大，性能足够）
        best_idx = 0
        best_diff = abs(self.gt_buffer[0][0] - t_query)
        
        for i, (t_gt, _, _) in enumerate(self.gt_buffer):
            diff = abs(t_gt - t_query)
            if diff < best_diff:
                best_diff = diff
                best_idx = i
        
        # 如果时间差超过 100ms，认为不匹配
        if best_diff > 0.1:
            return None, None
            
        _, x, y = self.gt_buffer[best_idx]
        return x, y

    def on_report(self):
        """生成报告和CSV"""
        if len(self.synced_data) < 30:
            self.get_logger().info(f"Collecting... synced_samples={len(self.synced_data)}")
            return

        # 转换为numpy数组便于计算
        data = self.synced_data
        t_arr = np.array([d['t'] for d in data])
        gt_x = np.array([d['gt_x'] for d in data])
        gt_y = np.array([d['gt_y'] for d in data])
        noisy_x = np.array([d['noisy_x'] for d in data])
        noisy_y = np.array([d['noisy_y'] for d in data])
        filt_x = np.array([d['filt_x'] for d in data])
        filt_y = np.array([d['filt_y'] for d in data])

        # 计算横向误差（相对于真值轨迹）
        xt_noisy = np.array([
            project_point_to_polyline(nx, ny, gt_x, gt_y) 
            for nx, ny in zip(noisy_x, noisy_y)
        ])
        xt_filt = np.array([
            project_point_to_polyline(fx, fy, gt_x, gt_y) 
            for fx, fy in zip(filt_x, filt_y)
        ])

        # 计算指标
        metrics = {
            'noisy': {
                'xt_rmse': float(np.sqrt(np.mean(xt_noisy**2))),
                'xt_p95': float(np.percentile(np.abs(xt_noisy), 95)),
                'pos_err': float(np.mean(np.sqrt((noisy_x - gt_x)**2 + (noisy_y - gt_y)**2)))
            },
            'filtered': {
                'xt_rmse': float(np.sqrt(np.mean(xt_filt**2))),
                'xt_p95': float(np.percentile(np.abs(xt_filt), 95)),
                'pos_err': float(np.mean(np.sqrt((filt_x - gt_x)**2 + (filt_y - gt_y)**2)))
            }
        }

        self.get_logger().info(
            f"\n[EVAL Report] Samples: {len(data)}\n"
            f"  Noisy  -> XT_RMSE: {metrics['noisy']['xt_rmse']:.4f}m | "
            f"Pos_Err: {metrics['noisy']['pos_err']:.4f}m\n"
            f"  UKF    -> XT_RMSE: {metrics['filtered']['xt_rmse']:.4f}m | "
            f"Pos_Err: {metrics['filtered']['pos_err']:.4f}m\n"
            f"  Improvement: {(1 - metrics['filtered']['xt_rmse']/metrics['noisy']['xt_rmse'])*100:.1f}%"
        )

        # 保存CSV（优化后的格式）
        if self.get_parameter('write_csv').value:
            csv_path = self.get_parameter('csv_path').value
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # 新的CSV格式：时间戳、真值、噪声输入、滤波输出、误差
                writer.writerow([
                    'timestamp', 
                    'gt_x', 'gt_y',           # Gazebo真值（黄金标准）
                    'noisy_x', 'noisy_y',     # 带噪声的odom输入
                    'filt_x', 'filt_y',       # UKF滤波输出
                    'xt_noisy', 'xt_filt'     # 横向误差（相对于真值轨迹）
                ])
                
                for i in range(len(data)):
                    writer.writerow([
                        f"{t_arr[i]:.6f}",
                        gt_x[i], gt_y[i],
                        noisy_x[i], noisy_y[i],
                        filt_x[i], filt_y[i],
                        xt_noisy[i], xt_filt[i]
                    ])
            self.get_logger().info(f"CSV saved: {csv_path} ({len(data)} samples)")

        # 保存图片
        if self.get_parameter('save_fig').value:
            self.save_figure(gt_x, gt_y, noisy_x, noisy_y, filt_x, filt_y)

    def save_figure(self, gt_x, gt_y, noisy_x, noisy_y, filt_x, filt_y):
        """保存可视化图片"""
        plt.figure(figsize=(12, 10))
        
        # 真值轨迹（黑色粗线）
        plt.plot(gt_x, gt_y, 'k-', linewidth=3, label='Ground Truth (Gazebo)', alpha=0.8)
        
        # 噪声数据（红色散点）
        plt.scatter(noisy_x, noisy_y, s=20, c='red', alpha=0.4, label='Noisy Odom', zorder=3)
        
        # UKF滤波（蓝色线+点）
        plt.plot(filt_x, filt_y, 'b-', linewidth=2, alpha=0.8, label='UKF Filtered')
        plt.scatter(filt_x, filt_y, s=30, c='blue', alpha=0.6, zorder=4)

        plt.axis('equal')
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend(fontsize=11, loc='best')
        plt.title('UKF Localization Evaluation\n(GT vs Noisy Input vs Filtered Output)', fontsize=14)
        plt.xlabel('X (m)', fontsize=12)
        plt.ylabel('Y (m)', fontsize=12)
        
        # 添加统计文本框
        textstr = f'Samples: {len(gt_x)}\n'
        plt.gcf().text(0.02, 0.02, textstr, fontsize=9, verticalalignment='bottom')
        
        plt.savefig(self.get_parameter('fig_path').value, dpi=200, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f"Figure saved: {self.get_parameter('fig_path').value}")


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