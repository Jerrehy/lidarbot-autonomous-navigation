"""
Простое избегание препятствий
Уровень 3: Базовая реализация
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Подписка на данные лидара
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Публикация команд скорости
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Параметры
        self.safe_distance = 0.5  # безопасное расстояние (метры)
        self.slow_distance = 1.0  # расстояние для замедления
        self.current_scan = None
        
        # Таймер управления
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Obstacle avoidance started')
        
    def scan_callback(self, msg):
        """Обработка данных лидара"""
        self.current_scan = msg
        
    def control_loop(self):
        """Основной цикл управления"""
        if self.current_scan is None:
            return
            
        cmd = Twist()
        
        # Анализ данных лидара
        ranges = np.array(self.current_scan.ranges)
        ranges = np.nan_to_num(ranges, nan=self.current_scan.range_max)
        
        # Разделение на сектора
        num_ranges = len(ranges)
        front_sector = ranges[num_ranges//3:2*num_ranges//3]
        left_sector = ranges[:num_ranges//3]
        right_sector = ranges[2*num_ranges//3:]
        
        min_front = np.min(front_sector)
        min_left = np.min(left_sector)
        min_right = np.min(right_sector)
        
        # Логика избегания препятствий
        if min_front < self.safe_distance:
            # Препятствие прямо - поворот
            if min_left > min_right:
                cmd.angular.z = -0.5  # поворот направо
            else:
                cmd.angular.z = 0.5   # поворот налево
            cmd.linear.x = 0.0
        elif min_front < self.slow_distance:
            # Замедление
            cmd.linear.x = 0.1
            cmd.angular.z = 0.0
        else:
            # Движение вперед
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Остановка робота
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
