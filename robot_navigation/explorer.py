#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random
import math

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("🤖 Автономный исследователь запущен (Random Walk)")
        self._client.wait_for_server()
        self.explore()

    def explore(self):
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        yaw = random.uniform(-3.14, 3.16)
        self.get_logger().info(f"🔍 Исследую точку: X={x:.2f}, Y={y:.2f}")
        
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self._client.send_goal_async(goal).add_done_callback(self._cb)

    def _cb(self, future):
        if future.result().accepted:
            future.result().get_result_async().add_done_callback(self._res)
        else:
            self.create_timer(2.0, self.explore) # Пробуем другую точку

    def _res(self, future):
        self.get_logger().info("✅ Точка достигнута или пропущена. Ищем новую...")
        self.create_timer(2.0, self.explore)

def main():
    rclpy.init()
    rclpy.spin(SimpleExplorer())
    rclpy.shutdown()
