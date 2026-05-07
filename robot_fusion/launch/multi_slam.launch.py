from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import launch

def generate_launch_description():
    
    # ========== 1. 三个机器人继续各自建图 ==========
    slam_params = {
        'use_sim_time': True,
        'mode': 'mapping',
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'message_filter_queue_length': 500,
        'minimum_time_interval': 0.1,
        'map_update_interval': 10.0,
    }
    
    robots = [
        {
            'name': 'rb1',
            'init_pose': [0.0, 0.0, 0.0],  # [x, y, yaw]
            'params': {**slam_params, 'base_frame': 'rb1/base_footprint', 
                      'odom_frame': 'rb1/odom', 'map_frame': 'rb1/map', 
                      'scan_topic': 'scan'}
        },
        {
            'name': 'rb2', 
            'init_pose': [0.0, 0.0, 0.0],  # 已知初始位置
            'params': {**slam_params, 'base_frame': 'rb2/base_footprint',
                      'odom_frame': 'rb2/odom', 'map_frame': 'rb2/map',
                      'scan_topic': 'scan'}
        },
        {
            'name': 'rb3',
            'init_pose': [0.0, 0.0, 0.0],  # 已知初始位置
            'params': {**slam_params, 'base_frame': 'rb3/base_footprint',
                      'odom_frame': 'rb3/odom', 'map_frame': 'rb3/map',
                      'scan_topic': 'scan'}
        },
    ]
    
    actions = []
    
    # 启动三个 SLAM 节点
    for robot in robots:
        actions.append(
            GroupAction([
                PushRosNamespace(robot['name']),
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[robot['params']],
                    remappings=[('/scan', 'scan'), ('/odom', 'odom'), ('/map', 'map')]
                ),
            ])
        )
    
    # ========== 2. 发布静态TF：将 rbX/map 对齐到 world 坐标系 ==========
    # 这是关键：通过静态变换把三个地图关联起来
    
    for robot in robots:
        x, y, yaw = robot['init_pose']
        # 将 world -> rbX/map 的变换发布出去
        # 注意：slam_toolbox发布的是 rbX/map -> rbX/odom
        # 我们需要发布 world -> rbX/map 来对齐三个地图
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f"{robot['name']}_map_tf",
                arguments=[
                    str(x), str(y), '0',  # x y z
                    str(yaw), '0', '0',   # yaw pitch roll (绕Z轴旋转)
                    'world',              # parent frame (全局地图)
                    f"{robot['name']}/map" # child frame (机器人本地地图)
                ]
            )
        )

    actions.append(
        # 添加到 launch 文件
        Node(
            package='robot_fusion',  # 你的包名
            executable='map_fusion_node',  # 或 component 方式
            name='map_fusion',
            output='screen',
            parameters=[{
                'world_frame': 'world',
                'merged_map_topic': '/merged_map',
                'map_resolution': 0.05,
                'update_rate': 2.0,
            }]
        )
    )
    
    return LaunchDescription(actions)