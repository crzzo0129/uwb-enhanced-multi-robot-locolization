from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tri_rb1base = Node(
        package='multi_robot_trilat',
        executable='tri_node_rb1base',
        name='tri_node_rb1base',
        output='screen',
        parameters=[
            {'pub_tf': False}
        ]
    )

    tri_rb2base = Node(
        package='multi_robot_trilat',
        executable='tri_node_rb2base',
        name='tri_node_rb2base',
        output='screen',
        parameters=[
            # 距离话题（PoseStamped: x=d12,y=d13,z=d23）
            {'distances_topic': '/robot_distances'},
            # rb2 作为参考系
            {'rb2_odom_topic': '/rb2/odom'},
            {'rb2_imu_topic': '/rb2/imu_plugin/out'},
            # A：首帧用已知初始
            {'init_known': True},
            {'init_rb1_xy': [0.0, 0.0]},
            {'init_rb2_xy': [0.0, 2.0]},
            {'init_rb3_xy': [1.0, 1.0]},
            {'init_rb2_yaw_deg': 0.0},

            {'flip_hysteresis': 2},
            {'y_deadzone': 0.05},
        ]
    )

    tri_rb3base = Node(
        package='multi_robot_trilat',
        executable='tri_node_rb3base',
        name='tri_node_rb3base',
        output='screen',
        parameters=[
            {'pub_tf': False}
        ]
    )
    return LaunchDescription([
        tri_rb1base,
        tri_rb2base,
        tri_rb3base,
    ])
