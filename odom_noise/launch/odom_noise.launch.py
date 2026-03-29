from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_noise',
            executable='odom_noise_node',
            name='odom_noise_rb1',
            output='screen',
            parameters=[
                {'in_topic':  '/rb1/odom'},
                {'out_topic': '/rb1/odom_noisy'},
                # 白噪声（位置、航向）
                {'sigma_x': 0.15},      # m
                {'sigma_y': 0.15},      # m
                {'sigma_yaw': 0.08},    # rad
                # 随机游走偏置强度（每秒）
                {'bias_rate_x': 0.02},     # m/sqrt(s)
                {'bias_rate_y': 0.02},     # m/sqrt(s)
                {'bias_rate_yaw': 0.006},   # rad/sqrt(s)
                {'bias_tau': 30.0},     # s, 越大越平滑
                {'noise_twist': False},
                {'seed': 12345},
            ]
        )
    ])
