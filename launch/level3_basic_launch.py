"""
Launch-файл для уровня 3: Базовая навигация
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Запуск драйвера лидара RPLIDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        ),
        
        # Запуск драйвера моторов
        Node(
            package='robot_navigation',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),
        
        # Запуск избегания препятствий
        Node(
            package='robot_navigation',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen'
        ),
    ])
