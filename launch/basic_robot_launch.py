from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Лидар
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', # Или /dev/rplidar, проверьте командой ls /dev/tty*
                'serial_baudrate': 115200,
                'frame_id': 'laser',
            }],
        ),
        # 4WD Драйвер моторов с правильными параметрами
        Node(
            package='robot_navigation',
            executable='motor_driver',
            name='motor_driver',
            parameters=[{
                'wheel_base': 0.30,
                'max_pwm': 1023,
                'max_speed': 0.5,
            }],
            output='screen',
        ),
        # IMU (ICM20948)
        Node(
            package='robot_navigation',
            executable='imu_driver',
            name='imu_driver',
            output='screen',
        ),
        # Статические TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
        ),
    ])
