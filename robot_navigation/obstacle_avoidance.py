"""
Простейшее избегание препятствий
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.safe_distance = 0.5  # метров
        
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
    
    def control_loop(self):
        cmd = Twist()
        
        if not hasattr(self, 'ranges'):
            return
        
        # Разделение на сектора
        num_ranges = len(self.ranges)
        sector_size = num_ranges // 3
        
        left = self.ranges[:sector_size]
        front = self.ranges[sector_size:2*sector_size]
        right = self.ranges[2*sector_size:]
        
        # Фильтрация бесконечных значений
        left_min = np.min(left[left > 0]) if np.any(left > 0) else float('inf')
        front_min = np.min(front[front > 0]) if np.any(front > 0) else float('inf')
        right_min = np.min(right[right > 0]) if np.any(right > 0) else float('inf')
        
        # Логика управления
        if front_min < self.safe_distance:
            # Препятствие впереди - поворот
            if left_min > right_min:
                cmd.angular.z = 0.5  # поворот влево
            else:
                cmd.angular.z = -0.5  # поворот вправо
        else:
            # Путь свободен - движение вперед
            cmd.linear.x = 0.2
        
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
