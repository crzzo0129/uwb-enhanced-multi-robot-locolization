from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ukf_rb1 = Node(
        package='robot_localization',
        executable='ukf_node',  # 修正可执行文件名称
        output='screen',
        namespace='rb1',
        parameters=['/home/rhw/fusion_ws/src/robot_fusion/config/ukf_rb1.yaml']
    )

    ukf_rb2= Node(
        package='robot_localization',
        executable='ukf_node',  # 修正可执行文件名称
        output='screen',
        namespace='rb2',
        parameters=['/home/rhw/fusion_ws/src/robot_fusion/config/ukf_rb2.yaml']
    )
    
    ukf_rb3 = Node(
        package='robot_localization',
        executable='ukf_node',  # 修正可执行文件名称
        output='screen',
        namespace='rb3',
        parameters=['/home/rhw/fusion_ws/src/robot_fusion/config/ukf_rb3.yaml']
    )

    return LaunchDescription([
        ukf_rb1,
        ukf_rb2,
        ukf_rb3,
    ])