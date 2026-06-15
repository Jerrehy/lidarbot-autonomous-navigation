#!/usr/bin/env python3
"""
Преобразует топик /odom в TF odom -> base_link
Необходимо для работы SLAM и Nav2
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info('✅ OdomToTf запущен. Ожидание данных /odom...')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # 'odom'
        t.child_frame_id = msg.child_frame_id    # 'base_link'
        
        # Позиция
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Ориентация (кватернион)
        t.transform.rotation = msg.pose.pose.orientation
        
        # Публикуем TF
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
