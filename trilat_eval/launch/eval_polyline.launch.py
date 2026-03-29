from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trilat_eval',            # 改成你放 eval_polyline_node.py 的包名
            executable='eval_polyline_node',  # 对应脚本名（去掉 .py）
            name='eval_polyline',
            output='screen',
            parameters=[
                {'gt_source': 'gazebo'},             # 'gazebo' 或 'path'
                {'gazebo_model_name': 'robot1'},     # Gazebo 模型名
                {'gt_path_topic': '/gt_path'},       # 若 gt_source=path 时使用
                {'odom_topic': '/rb1/odom_noisy'},
                {'filtered_topic': '/rb1/ukf_pose'},
                {'min_points': 100},
                {'report_period': 5.0},
                {'save_fig': True},
                {'fig_path': '/home/rhw/fusion_ws/src/trilat_eval/results/eval_polyline_rb1.png'},
                {'fig_size': [6.0, 6.0]},
                {'write_csv': True},
                {'csv_path': '/home/rhw/fusion_ws/src/trilat_eval/results/eval_polyline_rb1.csv'},
            ]
        )
    ])
