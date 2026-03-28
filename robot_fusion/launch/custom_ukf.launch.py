from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Launch custom UKF nodes for all three robots
    """
    
    # Get config directory
    config_dir = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config'
    )
    
    # Robot 1 UKF
    ukf_rb1 = Node(
        package='robot_fusion',
        executable='robot_ukf_node',
        name='ukf_rb1',
        namespace='rb1',
        output='screen',
        parameters=[
            {'robot_name': 'rb1'},
            os.path.join(config_dir, 'custom_ukf_rb1.yaml')
        ]
    )

    # Robot 2 UKF
    ukf_rb2 = Node(
        package='robot_fusion',
        executable='robot_ukf_node',
        name='ukf_rb2',
        namespace='rb2',
        output='screen',
        parameters=[
            {'robot_name': 'rb2'},
            os.path.join(config_dir, 'custom_ukf_rb2.yaml')
        ]
    )
    
    # Robot 3 UKF
    ukf_rb3 = Node(
        package='robot_fusion',
        executable='robot_ukf_node',
        name='ukf_rb3',
        namespace='rb3',
        output='screen',
        parameters=[
            {'robot_name': 'rb3'},
            os.path.join(config_dir, 'custom_ukf_rb3.yaml')
        ]
    )

    return LaunchDescription([
        ukf_rb1,
        ukf_rb2,
        ukf_rb3,
    ])
