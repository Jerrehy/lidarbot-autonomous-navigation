#!/usr/bin/env python3
"""
Планировщик миссий — последовательное прохождение точек из YAML файла.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import yaml
import os

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')

        # Объявление параметров
        self.declare_parameter('mission_file', '')
        self.declare_parameter('loop', False)

        mission_file = self.get_parameter('mission_file').value
        self.loop_mission = self.get_parameter('loop').value

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = []
        self.current_idx = 0

        # Загрузка миссии
        if mission_file and os.path.exists(mission_file):
            self.load_mission(mission_file)
        else:
            self.get_logger().error(f'❌ Файл миссии не найден: {mission_file}')
            rclpy.shutdown()
            return

        self.get_logger().info(f'✅ Миссия загружена: {len(self.waypoints)} точек')
        
        # Ожидание сервера навигации перед началом
        self.get_logger().info('⏳ Ожидание доступности сервера навигации Nav2...')
        self._action_client.wait_for_server()
        self.get_logger().info('🚀 Начало выполнения миссии!')
        self.send_next()

    def load_mission(self, filepath):
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f'Ошибка чтения файла {filepath}: {e}')
            rclpy.shutdown()

    def send_next(self):
        if self.current_idx >= len(self.waypoints):
            if self.loop_mission:
                self.current_idx = 0
                self.get_logger().info('🔄 Миссия завершена, начинаем заново...')
                self.create_timer(2.0, self.send_next)
            else:
                self.get_logger().info('🏁 Миссия полностью завершена!')
                rclpy.shutdown()
            return

        wp = self.waypoints[self.current_idx]
        self.get_logger().info(
            f'📍 Отправка к точке {self.current_idx + 1}/{len(self.waypoints)}: '
            f'X={wp["x"]}, Y={wp["y"]}, Yaw={wp["yaw"]} рад'
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.position.z = 0.0
        
        # Преобразование угла Yaw в кватернион
        yaw = float(wp['yaw'])
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Цель отклонена сервером навигации')
            self.current_idx += 1
            self.create_timer(2.0, self.send_next)
            return

        self.get_logger().info('✅ Цель принята, робот движется...')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info(f'🎉 Точка {self.current_idx + 1} успешно достигнута!')
        else:
            self.get_logger().warn(f'⚠️ Не удалось достичь точки {self.current_idx + 1} (статус: {status})')

        self.current_idx += 1
        # Небольшая пауза перед следующей точкой
        self.create_timer(2.0, self.send_next)

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
