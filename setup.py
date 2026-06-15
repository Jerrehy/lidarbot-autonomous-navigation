from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berry',
    maintainer_email='berry@example.com',
    description='Autonomous navigation for lidarbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Уровень 3 — базовые
            'motor_driver = robot_navigation.motor_driver:main',
            'imu_driver = robot_navigation.imu_driver:main',
            'odom_to_tf = robot_navigation.odom_to_tf:main',
            'remote_commander = robot_navigation.remote_commander:main',
            # Уровень 4 — расширенные
            'system_monitor = robot_navigation.system_monitor:main',
            'mission_planner = robot_navigation.mission_planner:main',
            # Уровень 5 — автономные
            'explorer = robot_navigation.explorer:main',
            'web_monitor = robot_navigation.web_monitor:main',
            'dynamic_obstacle_detector = robot_navigation.dynamic_obstacle_detector:main',
            'initial_pose_setter = robot_navigation.initial_pose_setter:main',
        ],
    },
)
