from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwbpsr_ratros2',
            executable='linktrack',
            name='linktrack0',
            output='screen',
            namespace = 'robot1',
            parameters=[
                {'port_name': '/dev/ttyUSB1'},
                {'baud_rate': 115200}
            ]
        )
    ])
    11
    1
    1
    1
    11
    2
    4
    24
    2
    4
    2
    4
    24
    42
    2
    42
    42
    42
    42
    42
    42
    42
    24
    42
    42
    42
    42
    42
    42
    42
    42
