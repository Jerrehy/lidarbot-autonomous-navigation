#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
import sys
import select
import tty
import termios

class RemoteCommander(Node):
    def __init__(self):
        super().__init__('remote_commander')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Состояние
        self.speed_linear = 0.2
        self.speed_angular = 0.5
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Подписка на одометрию для отображения позиции
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎮 REMOTE COMMANDER для LidarBot")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Управление: w(вперёд), x(назад), a(влево), d(вправо), s(стоп)")
        self.get_logger().info("Скорость: '+' (увеличить), '-' (уменьшить)")
        self.get_logger().info("Навигация: 'g' -> ввод X Y Yaw (относительно карты)")
        self.get_logger().info("Выход: 'q'")
        self.get_logger().info("=" * 60)

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def print_status(self):
        sys.stdout.write(f"\r📍 Pos: X:{self.current_x:5.2f} Y:{self.current_y:5.2f} Yaw:{math.degrees(self.current_yaw):5.1f}° | Speed: {self.speed_linear:.2f} м/с | Жду команду... ")
        sys.stdout.flush()

    def send_nav_goal(self, x, y, yaw):
        self.get_logger().info(f"\n🎯 Отправка цели: X={x}, Y={y}, Yaw={math.degrees(yaw):.1f}°")
        self._nav_client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2)
        
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.get_logger().info("✅ Цель принята" if f.result().accepted else "❌ Цель отклонена"))

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = RemoteCommander()
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.publish_cmd(0.0, 0.0) # Стоп при старте
        
        while rclpy.ok():
            node.print_status()
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == 'w': node.publish_cmd(node.speed_linear, 0.0)
                elif key == 'x': node.publish_cmd(-node.speed_linear, 0.0)
                elif key == 'a': node.publish_cmd(0.0, node.speed_angular)
                elif key == 'd': node.publish_cmd(0.0, -node.speed_angular)
                elif key == 's': node.publish_cmd(0.0, 0.0)
                elif key == '+': 
                    node.speed_linear = min(1.0, node.speed_linear + 0.1)
                    node.get_logger().info(f"Скорость увеличена: {node.speed_linear:.2f}")
                elif key == '-': 
                    node.speed_linear = max(0.1, node.speed_linear - 0.1)
                    node.get_logger().info(f"Скорость уменьшена: {node.speed_linear:.2f}")
                elif key == 'g':
                    node.publish_cmd(0.0, 0.0)
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    coords = input("\nВведите X Y Yaw(в градусах) через пробел: ").split()
                    tty.setcbreak(sys.stdin.fileno())
                    if len(coords) == 3:
                        node.send_nav_goal(float(coords[0]), float(coords[1]), math.radians(float(coords[2])))
                    else:
                        node.get_logger().error("Неверный формат!")
                elif key == 'q':
                    break
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
