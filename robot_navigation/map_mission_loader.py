#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import os
import math

class MapMissionLoader(Node):
    def __init__(self):
        super().__init__('map_mission_loader')
        self.declare_parameter('mission_name', 'default')
        mission_name = self.get_parameter('mission_name').value
        
        # Пути к файлам (адаптируйте под себя)
        base_dir = "/home/berry/nav_ws/src/lidarbot-autonomous-navigation/config"
        map_file = os.path.join(base_dir, f"{mission_name}_map.yaml")
        waypoints_file = os.path.join(base_dir, f"{mission_name}_waypoints.yaml")
        
        self.get_logger().info(f"Загрузка миссии: {mission_name}")
        
        # 1. Загрузка карты (через сервис map_server, упрощённо вызываем CLI или параметр)
        # В реальном лаунч-файле это делается через IncludeLaunchDescription с map:=...
        # Здесь мы просто читаем точки, предполагая, что карта уже загружена лаунч-файлом
        
        if not os.path.exists(waypoints_file):
            self.get_logger().error(f"Файл точек не найден: {waypoints_file}")
            rclpy.shutdown()
            return
            
        with open(waypoints_file, 'r') as f:
            self.waypoints = yaml.safe_load(f).get('waypoints', [])
            
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.idx = 0
        self.get_logger().info(f"Загружено {len(self.waypoints)} точек. Ожидание Nav2...")
        self._client.wait_for_server()
        self.send_next()

    def send_next(self):
        if self.idx >= len(self.waypoints):
            self.get_logger().info("🏁 Миссия завершена!")
            rclpy.shutdown()
            return
            
        wp = self.waypoints[self.idx]
        self.get_logger().info(f"➡️ Точка {self.idx+1}: X={wp['x']}, Y={wp['y']}")
        
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        goal.pose.pose.orientation.w = 1.0 # Упрощённо
        
        self._client.send_goal_async(goal).add_done_callback(self._response)

    def _response(self, future):
        if future.result().accepted:
            future.result().get_result_async().add_done_callback(self._result)
        else:
            self.get_logger().error("Цель отклонена")
            self.idx += 1
            self.send_next()

    def _result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("✅ Точка достигнута")
        else:
            self.get_logger().warn(f"⚠️ Сбой на точке (status={status})")
        self.idx += 1
        self.create_timer(1.0, self.send_next) # Пауза 1 сек

def main():
    rclpy.init()
    MapMissionLoader()
    rclpy.spin()

if __name__ == '__main__':
    main()
