from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berry',
    maintainer_email='berry@example.com',
    description='Autonomous navigation system for mobile robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = robot_navigation.motor_driver:main',
            'obstacle_avoidance = robot_navigation.obstacle_avoidance:main',
            'advanced_explorer = robot_navigation.advanced_explorer:main',
            'system_monitor = robot_navigation.system_monitor:main',
            # ЭТА СТРОКА КРИТИЧЕСКИ ВАЖНА ДЛЯ ПЛАНА Б:
            'fake_control = robot_navigation.fake_odometry_and_control:main',
        ],
    },
)
