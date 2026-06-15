"""
Запуск SLAM Toolbox для построения карты
Уровень 3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'slam_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
