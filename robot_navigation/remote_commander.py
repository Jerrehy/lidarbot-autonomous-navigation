#!/usr/bin/env python3
"""
Пульт управления с подробной диагностикой ввода координат
"""
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
import time

class RemoteCommander(Node):
    def __init__(self):
        super().__init__('remote_commander')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed_linear = 0.3
        self.speed_angular = 0.5
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.get_logger().info("=" * 65)
        self.get_logger().info("🎮 ПУЛЬТ УПРАВЛЕНИЯ LIDARBOT")
        self.get_logger().info("=" * 65)
        self.get_logger().info("Движение : w(вперед), x(назад), a(влево), d(вправо), s(стоп)")
        self.get_logger().info("Скорость : + (увеличить), - (уменьшить)")
        self.get_logger().info("Координаты: g (ввод цели X Y Yaw_в_градусах)")
        self.get_logger().info("Выход    : q")
        self.get_logger().info("=" * 65)
        self.get_logger().info("⏳ Ожидание подключения к серверу навигации (Nav2)...")
        
        # Ждем запуска Nav2
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ СЕРВЕР НАВИГАЦИИ НЕ НАЙДЕН!")
            self.get_logger().error("💡 Для ввода координат (команда 'g') ДОЛЖЕН быть запущен:")
            self.get_logger().error("   ros2 launch robot_navigation navigation_launch.py map:=...")
        else:
            self.get_logger().info("✅ Сервер навигации подключен. Можно использовать команду 'g'.")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def print_status(self):
        sys.stdout.write(f"\r📍 X:{self.x:5.2f} Y:{self.y:5.2f} Yaw:{math.degrees(self.yaw):5.1f}° | Скорость: {self.speed_linear:.2f} м/с | Команда: ")
        sys.stdout.flush()

    def send_nav_goal(self, x, y, yaw_deg):
        self.get_logger().info(f"\n🎯 ОТПРАВКА ЦЕЛИ: X={x}, Y={y}, Yaw={yaw_deg}°")
        
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map' # Цель всегда в системе координат карты!
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        
        yaw_rad = math.radians(float(yaw_deg))
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2)

        self.get_logger().info("⏳ Отправка цели на сервер...")
        future = self._nav_client.send_goal_async(goal)
        
        # Блокирующее ожидание ответа (чтобы вы видели результат в терминале)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Цель ОТКЛОНЕНА сервером навигации!")
            self.get_logger().error("💡 Возможные причины:")
            self.get_logger().error("   1. Не запущен navigation_launch.py")
            self.get_logger().error("   2. Не задан Fixed Frame 'map' в RViz2 / не построена карта")
            self.get_logger().error("   3. Точка находится в зоне препятствий (costmap)")
            return

        self.get_logger().info("✅ Цель ПРИНЯТА. Робот начал движение.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == 4:
            self.get_logger().info("🏁 ТОЧКА ДОСТИГНУТА УСПЕШНО!")
        else:
            self.get_logger().error(f"⚠️ НАВИГАЦИЯ ПРЕРВАНА (Статус: {status})")

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
        node.publish_cmd(0.0, 0.0)
        
        while rclpy.ok():
            node.print_status()
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == 'w': node.publish_cmd(node.speed_linear, 0.0)
                elif key == 'x': node.publish_cmd(-node.speed_linear, 0.0)
                elif key == 'a': node.publish_cmd(0.0, node.speed_angular)
                elif key == 'd': node.publish_cmd(0.0, -node.speed_angular)
                elif key == 's': 
                    node.publish_cmd(0.0, 0.0)
                    node.get_logger().info("\n🛑 СТОП")
                elif key == '+': 
                    node.speed_linear = min(1.0, node.speed_linear + 0.1)
                    node.get_logger().info(f"\n⚡ Скорость: {node.speed_linear:.2f} м/с")
                elif key == '-': 
                    node.speed_linear = max(0.1, node.speed_linear - 0.1)
                    node.get_logger().info(f"\n🐢 Скорость: {node.speed_linear:.2f} м/с")
                elif key == 'g':
                    node.publish_cmd(0.0, 0.0)
                    # Возвращаем нормальный режим терминала для ввода
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    user_input = input("\n📍 Введите координаты (X Y Yaw_в_градусах) через пробел: ")
                    # Возвращаем raw-режим
                    tty.setcbreak(sys.stdin.fileno())
                    
                    parts = user_input.strip().split()
                    if len(parts) == 3:
                        try:
                            node.send_nav_goal(float(parts[0]), float(parts[1]), float(parts[2]))
                        except ValueError:
                            node.get_logger().error("❌ Ошибка: введите числа!")
                    else:
                        node.get_logger().error("❌ Ошибка формата! Пример: 1.5 0.5 90")
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
