from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Teleop for rb1
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_rb1',
            output='screen',
            remappings=[('/cmd_vel', '/rb1/cmd_vel')]
        ),       
    ])