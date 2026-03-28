from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwbpsr_ratros2',
            executable='linktrack',
            name='linktrack0',
            output='screen',
            namespace = 'robot9',
            parameters=[
                {'port_name': '/dev/ttyUSB1'},
                {'baud_rate': 115200}
            ]
        )
    ])
