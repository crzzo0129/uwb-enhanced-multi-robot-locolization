import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_fusion')

    world_file = LaunchConfiguration('world_file')
    robot1_file = LaunchConfiguration('robot1_file')
    robot2_file = LaunchConfiguration('robot2_file')
    robot3_file = LaunchConfiguration('robot3_file')

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_share, 'models', 'world.sdf'),
        description='Path to the Gazebo world SDF file'
    )
    robot1_file_arg = DeclareLaunchArgument(
        'robot1_file',
        default_value=os.path.join(pkg_share, 'models', 'robot1.sdf'),
        description='Path to the SDF file of robot1'
    )
    robot2_file_arg = DeclareLaunchArgument(
        'robot2_file',
        default_value=os.path.join(pkg_share, 'models', 'robot2.sdf'),
        description='Path to the SDF file of robot2'
    )
    robot3_file_arg = DeclareLaunchArgument(
        'robot3_file',
        default_value=os.path.join(pkg_share, 'models', 'robot3.sdf'),
        description='Path to the SDF file of robot3'
    )

    gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--ros-args', '--param', 'use_sim_time:=true'],
    output='screen'
    )

    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', robot1_file,
            '-entity', 'robot1',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', robot2_file,
            '-entity', 'robot2',
            '-x', '0.0', '-y', '10.0', '-z', '0.0'
        ],
        output='screen'
    )

    spawn_robot3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', robot3_file,
            '-entity', 'robot3',
            '-x', '6.0', '-y', '8.0', '-z', '0.0'
        ],
        output='screen'
    )

    base_to_imu1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'rb1/base_link', 'rb1/imu_link'],
        output='screen'
    )
    base_to_imu2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'rb2/base_link', 'rb2/imu_link'],
        output='screen'
    )
    base_to_imu3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'rb3/base_link', 'rb3/imu_link'],
        output='screen'
    )

    return LaunchDescription([
        world_file_arg,
        robot1_file_arg,
        robot2_file_arg,
        robot3_file_arg,
        gazebo,
        spawn_robot1,
        spawn_robot2,
        spawn_robot3,
        base_to_imu1,
        base_to_imu2,
        base_to_imu3
    ])