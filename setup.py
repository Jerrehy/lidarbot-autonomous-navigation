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
        # Регистрация launch файлов (папку создадим ниже)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Регистрация config файлов
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berry',
    maintainer_email='berry@example.com',
    description='Autonomous navigation system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ВАЖНО: Здесь мы говорим ROS2, как запускать наши файлы
            'motor_driver = robot_navigation.motor_driver:main',
            'imu_driver = robot_navigation.imu_driver:main',
            'explorer = robot_navigation.explorer:main',
            'navigator = robot_navigation.navigator:main',
        ],
    },
)
