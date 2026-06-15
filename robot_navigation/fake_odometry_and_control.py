import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math

class FakeControlAndOdometry(Node):
    def __init__(self):
        super().__init__('fake_control_and_odometry')
        
        # Подписка на команды скорости
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Состояние робота
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # !!! ГЛАВНОЕ ИСПРАВЛЕНИЕ: Таймер для публикации 30 раз в секунду !!!
        # Даже если команд нет, мы продолжаем публиковать текущие (нулевые) координаты
        self.timer = self.create_timer(0.033, self.publish_state)
        
        self.get_logger().info('Fake Control & Odometry node started!')
        self.get_logger().info('Publishing /odom and TF at 30Hz continuously.')

    def cmd_vel_callback(self, msg):
        # Просто запоминаем последнюю полученную скорость
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def publish_state(self):
        # 1. Вычисляем прошедшее время
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 2. Интегрируем скорость для получения новой позиции
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.wz * dt
        
        # Нормализация угла в диапазон [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 3. Публикация TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Преобразование угла в кватернион (только по оси Z для 2D)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        # 4. Публикация сообщения Odometry
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = FakeControlAndOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()	
