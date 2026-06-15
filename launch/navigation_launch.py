"""
Запуск полного стека Nav2
Уровень 3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_pkg = get_package_share_directory('nav2_bringup')
    pkg = get_package_share_directory('robot_navigation')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Full path to map yaml file to load'
    )
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    # Включаем стандартные launch Nav2
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'localization_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    return LaunchDescription([
        map_file_arg,
        use_sim_time,
        localization_launch,
        navigation_launch,
    ])
