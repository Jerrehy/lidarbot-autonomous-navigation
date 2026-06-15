"""
Удалённое управление роботом через ROS2 CLI
Позволяет отправлять цели навигации через SSH-терминал
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
import math
import sys
import select
import tty
import termios


class RemoteCommander(Node):
    def __init__(self):
        super().__init__('remote_commander')

        # Action client для Nav2
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Прямая публикация cmd_vel для ручного управления
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('=' * 50)
        self.get_logger().info('REMOTE COMMANDER для lidarbot')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Команды:')
        self.get_logger().info('  w/x — вперёд/назад')
        self.get_logger().info('  a/d — поворот влево/вправо')
        self.get_logger().info('  s — стоп')
        self.get_logger().info('  g x y yaw — навигация к точке (например: g 2.0 1.0 0.0)')
        self.get_logger().info('  q — выход')
        self.get_logger().info('=' * 50)

        self.speed_linear = 0.2
        self.speed_angular = 0.5

    def send_nav_goal(self, x, y, yaw):
        """Отправка цели навигации"""
        self.get_logger().info(f'Ожидание сервера навигации...')
        self._nav_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2)

        self.get_logger().info(f'Отправка цели: x={x}, y={y}, yaw={yaw}')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Цель отклонена!')
            return
        self.get_logger().info('Цель принята, навигация начата...')
        goal_handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('✅ Точка достигнута!')
        else:
            self.get_logger().error(f'❌ Навигация провалена (status={status})')

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = RemoteCommander()

    # Сохраняем настройки терминала
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            # Проверяем, есть ли ввод
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)

                if key == 'w':
                    node.publish_cmd(node.speed_linear, 0.0)
                    node.get_logger().info('⬆️ Вперёд')
                elif key == 'x':
                    node.publish_cmd(-node.speed_linear, 0.0)
                    node.get_logger().info('⬇️ Назад')
                elif key == 'a':
                    node.publish_cmd(0.0, node.speed_angular)
                    node.get_logger().info('⬅️ Влево')
                elif key == 'd':
                    node.publish_cmd(0.0, -node.speed_angular)
                    node.get_logger().info('➡️ Вправо')
                elif key == 's':
                    node.publish_cmd(0.0, 0.0)
                    node.get_logger().info('🛑 СТОП')
                elif key == 'g':
                    # Читаем координаты
                    sys.stdout.write('Введите x y yaw: ')
                    sys.stdout.flush()
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    coords = input().split()
                    tty.setcbreak(sys.stdin.fileno())
                    if len(coords) == 3:
                        node.send_nav_goal(*coords)
                    else:
                        node.get_logger().error('Формат: g x y yaw')
                elif key == 'q':
                    node.get_logger().info('Выход...')
                    break

            rclpy.spin_once(node, timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.publish_cmd(0.0, 0.0)  # Стоп
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
