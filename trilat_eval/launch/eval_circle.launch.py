from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trilat_eval',
            executable='eval_circle_node',
            name='eval_circle',
            output='screen',
            parameters=[
                {'robot_name': 'rb1'},
                {'gt_entity':  'robot1'},
                {'gt_topic':   '/model_states'},
                {'odom_topic': '/rb1/odom_noisy'},
                {'filt_topic': '/rb1/odometry/filtered'},
                {'window_sec': 60.0},
                {'report_period': 5.0},
                {'min_points': 100},
                {'save_csv': False},
                {'csv_path': '/home/rhw/fusion_ws/src/trilat_eval/results/trilat_eval_rb1.csv'},
                {'save_fig': True},
                {'fig_path': '/home/rhw/fusion_ws/src/trilat_eval/results/trilat_eval_rb1.png'},
                {'fig_size': [6.0, 6.0]},
            ]
        )
    ])
