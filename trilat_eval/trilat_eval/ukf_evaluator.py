#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UKF离线评估脚本 - 纯NumPy实现，无SciPy依赖
优化版：使用高斯卷积实现平滑（兼容所有NumPy版本）
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
import math
from typing import Dict, Tuple, List, Optional
from collections import defaultdict

class UKFOfflineEvaluator:
    def __init__(self, csv_path: str):
        # 使用标准库csv读取数据
        self.data = defaultdict(list)

        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                for key, value in row.items():
                    self.data[key].append(float(value))

        # 转换为numpy数组
        self.t = np.array(self.data['t'])
        self.noisy = np.column_stack([
            np.array(self.data['noisy_x']), 
            np.array(self.data['noisy_y'])
        ])
        self.filtered = np.column_stack([
            np.array(self.data['filt_x']), 
            np.array(self.data['filt_y'])
        ])

        if 'odom_x' in self.data and 'odom_y' in self.data:
            self.gt = np.column_stack([
                np.array(self.data['odom_x']),
                np.array(self.data['odom_y'])
            ])
        else:
            self.gt = None

        print(f"Loaded {len(self.t)} data points")
        print(f"Time range: {self.t[0]:.2f} to {self.t[-1]:.2f} seconds")

    def compute_ate(self) -> Dict[str, float]:
        """Absolute Trajectory Error"""
        noisy_aligned = self.noisy - self.noisy[0]
        filtered_aligned = self.filtered - self.filtered[0]
        errors = np.linalg.norm(noisy_aligned - filtered_aligned, axis=1)

        return {
            'rmse': np.sqrt(np.mean(errors**2)),
            'mean': np.mean(errors),
            'std': np.std(errors),
            'max': np.max(errors),
            'p95': np.percentile(errors, 95),
            'p99': np.percentile(errors, 99),
            'median': np.median(errors)
        }

    def compute_smoothness(self) -> Dict[str, float]:
        """计算轨迹平滑度（二阶差分）"""
        noisy_2nd = np.diff(self.noisy, n=2, axis=0)
        filtered_2nd = np.diff(self.filtered, n=2, axis=0)

        noisy_smooth = np.mean(np.linalg.norm(noisy_2nd, axis=1))
        filtered_smooth = np.mean(np.linalg.norm(filtered_2nd, axis=1))
        improvement = (noisy_smooth - filtered_smooth) / noisy_smooth * 100

        return {
            'noisy_smoothness': noisy_smooth,
            'filtered_smoothness': filtered_smooth,
            'improvement_percent': improvement,
            'ratio': noisy_smooth / filtered_smooth if filtered_smooth > 0 else 0
        }

    def compute_drift_statistics(self) -> Dict[str, float]:
        """长期漂移统计"""
        duration = self.t[-1] - self.t[0]

        dist_noisy = np.sum(np.linalg.norm(np.diff(self.noisy, axis=0), axis=1))
        dist_filtered = np.sum(np.linalg.norm(np.diff(self.filtered, axis=0), axis=1))

        straight_dist_noisy = np.linalg.norm(self.noisy[-1] - self.noisy[0])
        straight_dist_filtered = np.linalg.norm(self.filtered[-1] - self.filtered[0])

        efficiency_noisy = straight_dist_noisy / dist_noisy if dist_noisy > 0 else 0
        efficiency_filtered = straight_dist_filtered / dist_filtered if dist_filtered > 0 else 0

        return {
            'total_time': duration,
            'path_length_noisy': dist_noisy,
            'path_length_filtered': dist_filtered,
            'displacement_noisy': straight_dist_noisy,
            'displacement_filtered': straight_dist_filtered,
            'path_efficiency_noisy': efficiency_noisy,
            'path_efficiency_filtered': efficiency_filtered,
            'drift_rate_noisy': straight_dist_noisy / duration if duration > 0 else 0,
            'drift_rate_filtered': straight_dist_filtered / duration if duration > 0 else 0
        }

    def compute_error_statistics(self) -> Dict[str, Dict]:
        """计算误差统计"""
        if 'xt_odom' in self.data:
            xt_noisy = np.array(self.data['xt_odom'])
            xt_filt = np.array(self.data['xt_filt'])

            return {
                'noisy_cross_track': {
                    'rmse': np.sqrt(np.mean(xt_noisy**2)),
                    'mean': np.mean(np.abs(xt_noisy)),
                    'max': np.max(np.abs(xt_noisy)),
                    'std': np.std(xt_noisy)
                },
                'filtered_cross_track': {
                    'rmse': np.sqrt(np.mean(xt_filt**2)),
                    'mean': np.mean(np.abs(xt_filt)),
                    'max': np.max(np.abs(xt_filt)),
                    'std': np.std(xt_filt)
                },
                'improvement': (np.mean(np.abs(xt_noisy)) - np.mean(np.abs(xt_filt))) / np.mean(np.abs(xt_noisy)) * 100
            }
        else:
            diff = np.linalg.norm(self.noisy - self.filtered, axis=1)
            return {
                'trajectory_difference': {
                    'rmse': np.sqrt(np.mean(diff**2)),
                    'mean': np.mean(diff),
                    'max': np.max(diff)
                }
            }

    def compute_position_accuracy(self) -> Dict[str, float]:
        """位置精度分析"""
        errors = np.linalg.norm(self.noisy - self.filtered, axis=1)

        return {
            'mean_error': np.mean(errors),
            'rmse': np.sqrt(np.mean(errors**2)),
            'std': np.std(errors),
            'max_error': np.max(errors),
            'p95': np.percentile(errors, 95),
            'p99': np.percentile(errors, 99)
        }

    @staticmethod
    def gaussian_smooth(data: np.ndarray, sigma: float = 2.0, mode: str = 'edge') -> np.ndarray:
        """
        使用高斯核卷积进行平滑（纯NumPy实现，无SciPy依赖）

        Args:
            data: 输入数据，1D或2D数组
            sigma: 高斯核标准差，越大越平滑
            mode: 边界处理模式，'nearest'最近邻填充

        Returns:
            平滑后的数据
        """
        if sigma <= 0:
            return data

        # 生成高斯核
        kernel_size = int(6 * sigma)  # 覆盖99%能量
        if kernel_size % 2 == 0:
            kernel_size += 1
        if kernel_size < 3:
            kernel_size = 3

        x = np.arange(kernel_size) - kernel_size // 2
        kernel = np.exp(-(x**2) / (2 * sigma**2))
        kernel = kernel / np.sum(kernel)  # 归一化

        if data.ndim == 1:
            # 一维数据
            # 边界填充
            pad_width = kernel_size // 2
            padded = np.pad(data, pad_width, mode='edge')
            # 卷积
            result = np.convolve(padded, kernel, mode='valid')
            # 确保长度一致
            if len(result) != len(data):
                result = result[:len(data)]
            return result
        else:
            # 二维数据（如XY轨迹），分别对每列平滑
            result = np.zeros_like(data)
            for i in range(data.shape[1]):
                pad_width = kernel_size // 2
                padded = np.pad(data[:, i], pad_width, mode='edge')
                smoothed = np.convolve(padded, kernel, mode='valid')
                if len(smoothed) != len(data):
                    smoothed = smoothed[:len(data)]
                result[:, i] = smoothed
            return result

    @staticmethod
    def interpolate_time(t_orig: np.ndarray, data: np.ndarray, num_points: int = 500) -> Tuple[np.ndarray, np.ndarray]:
        """
        时间序列插值（线性插值，纯NumPy）

        Args:
            t_orig: 原始时间数组
            data: 原始数据
            num_points: 目标插值点数

        Returns:
            t_new: 新的时间数组
            data_new: 插值后的数据
        """
        t_new = np.linspace(t_orig[0], t_orig[-1], num_points)

        if data.ndim == 1:
            data_new = np.interp(t_new, t_orig, data)
        else:
            data_new = np.zeros((num_points, data.shape[1]))
            for i in range(data.shape[1]):
                data_new[:, i] = np.interp(t_new, t_orig, data[:, i])

        return t_new, data_new

    def _smooth_trajectory(self, t: np.ndarray, data: np.ndarray, 
                          num_points: int = 500, sigma: float = 2.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        轨迹平滑：先高斯平滑，再插值

        Args:
            t: 时间数组
            data: 数据（N,2）或（N,）
            num_points: 插值点数
            sigma: 高斯平滑核大小（数据点数的倍数），默认2.0

        Returns:
            t_smooth, data_smooth
        """
        # 先进行高斯平滑（时域滤波）
        if sigma > 0:
            data_smooth_temp = self.gaussian_smooth(data, sigma=sigma)
        else:
            data_smooth_temp = data

        # 然后插值到更密集的采样点（仅用于视觉显示）
        t_smooth, data_smooth = self.interpolate_time(t, data_smooth_temp, num_points)

        return t_smooth, data_smooth

    def plot_analysis(self, save_path: str = None, smooth_sigma: Optional[float] = None):
        """
        绘制分析图表 - 使用纯NumPy高斯平滑

        Args:
            save_path: 保存路径
            smooth_sigma: 高斯平滑核大小（数据点数），默认2.0，越大越平滑，0表示不平滑
        """
        # 自动选择平滑参数
        if smooth_sigma is None:
            smooth_sigma = max(2.0, len(self.t) * 0.005)  # 自适应：约0.5%数据长度

        fig, axes = plt.subplots(2, 3, figsize=(15, 10))

        # 生成平滑的UKF轨迹（仅用于显示，原始数据保留）
        t_smooth, filtered_smooth = self._smooth_trajectory(
            self.t, self.filtered, num_points=500, sigma=smooth_sigma
        )

        # 1. 轨迹对比（XY平面）
        ax1 = axes[0, 0]
        # 原始噪声数据（非常细的半透明线）
        ax1.plot(self.noisy[:, 0], self.noisy[:, 1], 'r-', alpha=0.9, label='Noisy Odom', linewidth=2.0)
        # UKF平滑轨迹（粗线）
        ax1.plot(filtered_smooth[:, 0], filtered_smooth[:, 1], 'b-', alpha=0.9, 
                label='UKF Filtered (Smooth)', linewidth=2.5)
        # 显示部分原始UKF点（表明数据密度）
        skip = max(1, len(self.filtered) // 50)  # 显示约50个点
        ax1.scatter(self.filtered[::skip, 0], self.filtered[::skip, 1], 
                   c='blue', s=20, alpha=0.4, zorder=5, edgecolors='white', linewidth=0.5)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory Comparison')
        ax1.legend(loc='best', fontsize=9)
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # 2. X方向时序
        ax2 = axes[0, 1]
        t_s, x_smooth = self._smooth_trajectory(self.t, self.filtered[:, 0], num_points=400, sigma=smooth_sigma)
        ax2.plot(self.t, self.noisy[:, 0], 'r-', alpha=0.3, linewidth=0.8, label='Noisy')
        ax2.plot(t_s, x_smooth, 'b-', alpha=0.9, linewidth=2, label='UKF (Smooth)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('X (m)')
        ax2.set_title('X Position vs Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # 3. Y方向时序
        ax3 = axes[0, 2]
        _, y_smooth = self._smooth_trajectory(self.t, self.filtered[:, 1], num_points=400, sigma=smooth_sigma)
        ax3.plot(self.t, self.noisy[:, 1], 'r-', alpha=0.3, linewidth=0.8, label='Noisy')
        ax3.plot(t_s, y_smooth, 'b-', alpha=0.9, linewidth=2, label='UKF (Smooth)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Y (m)')
        ax3.set_title('Y Position vs Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # 4. 误差分析
        ax4 = axes[1, 0]
        errors = np.linalg.norm(self.noisy - self.filtered, axis=1)
        # 对误差做轻微平滑以便观察趋势
        t_err, errors_smooth = self._smooth_trajectory(
            self.t, errors, num_points=300, sigma=max(1.0, smooth_sigma*0.5)
        )
        ax4.plot(t_err, errors_smooth, 'g-', alpha=0.8, linewidth=1.5, label='Error Trend')
        ax4.fill_between(t_err, errors_smooth, alpha=0.2, color='green')
        ax4.axhline(y=np.mean(errors), color='r', linestyle='--', label=f'Mean: {np.mean(errors):.3f}m')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Error (m)')
        ax4.set_title('Tracking Error (Noisy vs Filtered)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        # 5. 误差直方图
        ax5 = axes[1, 1]
        ax5.hist(errors, bins=50, color='green', alpha=0.7, edgecolor='black')
        ax5.axvline(x=np.mean(errors), color='r', linestyle='--', label=f'Mean: {np.mean(errors):.3f}m')
        ax5.axvline(x=np.median(errors), color='orange', linestyle='--', label=f'Median: {np.median(errors):.3f}m')
        ax5.set_xlabel('Error Magnitude (m)')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Error Distribution')
        ax5.legend()
        ax5.grid(True, alpha=0.3)

        # 6. 速度分析
        ax6 = axes[1, 2]
        dt = np.diff(self.t)
        vx_noisy = np.diff(self.noisy[:, 0]) / dt
        vy_noisy = np.diff(self.noisy[:, 1]) / dt
        vx_filt = np.diff(self.filtered[:, 0]) / dt
        vy_filt = np.diff(self.filtered[:, 1]) / dt

        speed_noisy = np.sqrt(vx_noisy**2 + vy_noisy**2)
        speed_filt = np.sqrt(vx_filt**2 + vy_filt**2)
        t_speed = self.t[:-1]

        # 对速度做平滑
        _, speed_filt_smooth = self._smooth_trajectory(
            t_speed, speed_filt, num_points=300, sigma=max(3.0, smooth_sigma*1.5)
        )
        t_speed_smooth = np.linspace(t_speed[0], t_speed[-1], 300)

        ax6.plot(t_speed, speed_noisy, 'r-', alpha=0.3, linewidth=0.8, label='Noisy Speed')
        ax6.plot(t_speed_smooth, speed_filt_smooth, 'b-', alpha=0.9, linewidth=2, label='UKF Speed (Smooth)')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Speed (m/s)')
        ax6.set_title('Estimated Speed')
        ax6.legend()
        ax6.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Plot saved to {save_path}")
        else:
            plt.show()

    def generate_report(self) -> str:
        """生成文本报告"""
        report = []
        report.append("=" * 60)
        report.append("UKF Performance Evaluation Report (NumPy Only)")
        report.append("=" * 60)

        report.append(f"\n[Data Summary]")
        report.append(f"  Total samples: {len(self.t)}")
        report.append(f"  Duration: {self.t[-1] - self.t[0]:.2f} seconds")

        ate = self.compute_ate()
        report.append(f"\n[1] Trajectory Error (Relative to UKF)")
        report.append(f"  RMSE:  {ate['rmse']:.4f} m")
        report.append(f"  Mean:  {ate['mean']:.4f} m")
        report.append(f"  Std:   {ate['std']:.4f} m")
        report.append(f"  P95:   {ate['p95']:.4f} m")
        report.append(f"  Max:   {ate['max']:.4f} m")

        smooth = self.compute_smoothness()
        report.append(f"\n[2] Smoothness Analysis (2nd derivative)")
        report.append(f"  Noisy trajectory:   {smooth['noisy_smoothness']:.4f}")
        report.append(f"  Filtered trajectory: {smooth['filtered_smoothness']:.4f}")
        report.append(f"  Improvement: {smooth['improvement_percent']:.1f}%")
        report.append(f"  Smoothness ratio: {smooth['ratio']:.2f}x")

        drift = self.compute_drift_statistics()
        report.append(f"\n[3] Drift Analysis")
        report.append(f"  Path length (Noisy):  {drift['path_length_noisy']:.2f} m")
        report.append(f"  Path length (UKF):    {drift['path_length_filtered']:.2f} m")
        report.append(f"  Displacement (Noisy): {drift['displacement_noisy']:.2f} m")
        report.append(f"  Displacement (UKF):   {drift['displacement_filtered']:.2f} m")
        report.append(f"  Path efficiency (Noisy): {drift['path_efficiency_noisy']:.3f}")
        report.append(f"  Path efficiency (UKF):   {drift['path_efficiency_filtered']:.3f}")
        report.append(f"  Drift rate (Noisy):  {drift['drift_rate_noisy']:.4f} m/s")
        report.append(f"  Drift rate (UKF):    {drift['drift_rate_filtered']:.4f} m/s")

        err_stats = self.compute_error_statistics()
        report.append(f"\n[4] Error Statistics")
        if 'noisy_cross_track' in err_stats:
            nxt = err_stats['noisy_cross_track']
            fxt = err_stats['filtered_cross_track']
            report.append(f"  Cross-track RMSE (Noisy):  {nxt['rmse']:.4f} m")
            report.append(f"  Cross-track RMSE (UKF):    {fxt['rmse']:.4f} m")
            report.append(f"  Cross-track improvement:   {err_stats['improvement']:.1f}%")
        else:
            td = err_stats.get('trajectory_difference', {})
            report.append(f"  Trajectory deviation RMSE: {td.get('rmse', 0):.4f} m")

        pos_acc = self.compute_position_accuracy()
        report.append(f"\n[5] Position Accuracy (Noisy vs Filtered)")
        report.append(f"  Mean deviation: {pos_acc['mean_error']:.4f} m")
        report.append(f"  RMSE:           {pos_acc['rmse']:.4f} m")
        report.append(f"  Std deviation:  {pos_acc['std']:.4f} m")
        report.append(f"  Max deviation:  {pos_acc['max_error']:.4f} m")
        report.append(f"  P95:            {pos_acc['p95']:.4f} m")

        report.append("\n" + "=" * 60)
        return "\n".join(report)

def main():
    # 修改为你的CSV路径
    csv_file = "/home/rhw/fusion_ws/src/trilat_eval/results/eval_polyline_rb1.csv"

    try:
        evaluator = UKFOfflineEvaluator(csv_file)

        # 生成并打印报告
        report = evaluator.generate_report()
        print(report)

        # 保存报告
        report_path = csv_file.replace('.csv', '_report.txt')
        with open(report_path, 'w') as f:
            f.write(report)
        print(f"\nReport saved to: {report_path}")

        # 绘图（使用纯NumPy高斯平滑，无需SciPy）
        # smooth_sigma: 平滑核大小（标准差，单位：数据点数），默认自动计算
        plot_path = csv_file.replace('.csv', '_analysis.png')
        evaluator.plot_analysis(plot_path, smooth_sigma=0)  # None表示自动

        print(f"\nPlot saved to: {plot_path}")
        print("\nTip: 如果曲线过于平滑/不够平滑，调整 plot_analysis 中的 smooth_sigma 参数")
        print("     - 数值越大越平滑（如 5.0, 10.0）")
        print("     - 数值越小越接近原始数据（如 0.5, 1.0）")
        print("     - 设为 0 则完全不平滑，显示原始UKF轨迹")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
