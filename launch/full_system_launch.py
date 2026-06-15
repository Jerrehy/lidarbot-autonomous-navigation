from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 1. Объявляем аргументы, которые можно передать из командной строки
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Полный путь к файлу карты .yaml'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Использовать время симуляции (Gazebo), если true'
    )

    # 2. Путь к нашим параметрам Nav2
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # 3. Включаем ОФИЦИАЛЬНЫЙ bringup от Nav2. 
    # Он сам корректно обработает аргумент 'map' и запустит map_server, AMCL и planner
    nav2_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
        'map': LaunchConfiguration('map'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': nav2_params,
        'autostart': 'true',
        # Начальная поза: X=0, Y=0, Yaw=0 (в центре карты)
        'params_file': nav2_params,}.items()
    )

    # Добавляем ноду, которая автоматически задаёт начальную позу через 3 секунды после запуска
    initial_pose_setter = Node(
    package='robot_navigation',
    executable='initial_pose_setter',
    name='initial_pose_setter',
    output='screen',
    parameters=[{
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,
        'delay': 3.0, }]
    )

    # 4. Добавляем наши кастомные ноды 5-го уровня
    system_monitor = Node(
        package='robot_navigation',
        executable='system_monitor',
        name='system_monitor',
        output='screen'
    )

    web_monitor = Node(
        package='robot_navigation',
        executable='web_monitor',
        name='web_monitor',
        output='screen'
    )

    dynamic_obstacle_detector = Node(
        package='robot_navigation',
        executable='dynamic_obstacle_detector',
        name='dynamic_obstacle_detector',
        output='screen'
    )

    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        nav2_bringup,
        system_monitor,
        web_monitor,
        dynamic_obstacle_detector,
    ])
