from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Teleop for rb2
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_rb2',
            output='screen',
            remappings=[('/cmd_vel', '/rb2/cmd_vel')]
        ),       
    ])