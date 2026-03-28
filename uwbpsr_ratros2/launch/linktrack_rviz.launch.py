from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # linktrack_rviz_converter node
        Node(
            package='uwbpsr_ratros2',
            executable='linktrack_rviz_converter',
            name='linktrack_rviz_converter0',
            output='screen',
            parameters=[
                {'map_frame': '/linktrack_map'}
            ]
        ),

        # static_transform_publisher (tf2)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base2linktrack',
            arguments=['0', '0', '0', '0', '0', '0', '1', '/map', '/linktrack_map']
        ),

        # Launch RViz with config file
        ExecuteProcess(
            cmd=[
                'rviz2',
                '-d',
                '/opt/ros/humble/share/uwbpsr_ratros2/rviz/linktrack.rviz'
            ],
            output='screen'
        )
    ])
