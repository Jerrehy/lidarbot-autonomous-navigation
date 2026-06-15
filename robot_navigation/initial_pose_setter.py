#!/usr/bin/env python3
"""
Автоматически задаёт начальную позу робота в AMCL.
Убирает необходимость кликать '2D Pose Estimate' в RViz2.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Ждём 3 секунды, чтобы AMCL и map_server успели запуститься
        self.timer = self.create_timer(3.0, self.set_pose)

    def set_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Начальные координаты (центр карты)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.z = 0.0  # sin(0/2)
        msg.pose.pose.orientation.w = 1.0  # cos(0/2)
        
        # Небольшая ковариация, чтобы AMCL мог корректировать позицию
        msg.pose.covariance[0] = 0.25   # X
        msg.pose.covariance[7] = 0.25   # Y
        msg.pose.covariance[35] = 0.1   # Yaw
        
        self.pub.publish(msg)
        self.get_logger().info('✅ Начальная поза (0, 0, 0) отправлена в AMCL. Фрейм map создан.')
        
        # Останавливаем ноду после отправки
        self.timer.cancel()
        self.create_timer(1.0, lambda: rclpy.shutdown())

def main():
    rclpy.init()
    node = InitialPoseSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
