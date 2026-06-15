import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    nav2_config = os.path.join(pkg_robot_navigation, 'config', 'nav2_params.yaml')
    urdf_file = os.path.join(pkg_robot_navigation, 'urdf', 'lidarbot.urdf.xacro')
    
    # Генерируем URDF
    robot_description = Command(['xacro ', urdf_file])
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # 1. Robot State Publisher (статические TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )
    
    # 2. НАША НОДА (Заменяет и ros2_control, и EKF). Публикует /odom и TF odom->base_link
    fake_control_node = Node(
        package='robot_navigation',
        executable='fake_control',
        name='fake_control_and_odometry',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 3. RPLIDAR
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )
    
    # 4. Nav2 Bringup (ВКЛЮЧАЕТ SLAM TOOLBOX ВНУТРИ СЕБЯ)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',          # <-- Ключевой параметр: запускает SLAM
            'map': '',               # <-- Пустая карта, мы строим новую
            'params_file': nav2_config,
        }.items(),
    )
    
    # 5. RViz2 (только если нужно)
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_state_publisher,
        fake_control_node,
        rplidar_node,
        nav2_bringup,
        rviz_node,
    ])
