"""
Базовый запуск: моторы + IMU + лидар
Уровень 3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')

    return LaunchDescription([
        # Лидар RPLIDAR A1
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[('/scan', '/scan')]
        ),

        # Драйвер моторов
        Node(
            package='robot_navigation',
            executable='motor_driver',
            name='motor_driver',
            parameters=[{
                'left_forward_pin': 17,
                'left_backward_pin': 27,
                'left_pwm_pin': 12,
                'right_forward_pin': 22,
                'right_backward_pin': 23,
                'right_pwm_pin': 13,
                'wheel_base': 0.30,
                'wheel_radius': 0.05,
                'max_pwm': 1023,
                'max_speed': 0.5,
            }],
            output='screen',
        ),

        # Драйвер IMU
        Node(
            package='robot_navigation',
            executable='imu_driver',
            name='imu_driver',
            output='screen',
        ),

        # Статический TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.1', '0.0', '0.15', '0.0', '0.0', '0.0',
                       'base_link', 'laser']
        ),

        # Статический TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0',
                       'base_link', 'imu_link']
        ),
    ])
