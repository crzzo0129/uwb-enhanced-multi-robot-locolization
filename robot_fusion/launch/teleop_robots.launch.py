from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_rb1',
            output='screen',
            prefix='xterm -e',  # 在新终端中运行，便于键盘输入
            remappings=[('/cmd_vel', '/rb1/cmd_vel')]
        ),
        
        
    ])
