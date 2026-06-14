"""
Навигация к заданным точкам
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        
        # Action client для навигации
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        
        # Целевые точки (пример)
        self.waypoints = [
            (1.0, 0.0, 0.0),    # x, y, yaw
            (2.0, 1.0, 1.57),
            (3.0, 0.0, 3.14),
            (2.0, -1.0, -1.57),
            (0.0, 0.0, 0.0)
        ]
        
        self.current_waypoint_index = 0
        
        self.get_logger().info('Navigator initialized')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        
        # Запуск навигации
        self.send_next_goal()
    
    def send_next_goal(self):
        """Отправка следующей цели"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            return
        
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Создание цели
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        x, y, yaw = self.waypoints[self.current_waypoint_index]
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        
        # Преобразование yaw в кватернион
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.get_logger().info(
            f'Sending goal {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )
        
        # Отправка цели
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Обработка ответа на goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Обработка результата"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
            self.current_waypoint_index += 1
            
            # Переход к следующей точке через 2 секунды
            self.create_timer(2.0, self.send_next_goal)
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
    
    def feedback_callback(self, feedback_msg):
        """Обработка feedback"""
        feedback = feedback_msg.feedback
        # Можно использовать для отслеживания прогресса
        pass

def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
