"""
Обнаружение динамических препятствий
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from collections import deque

class DynamicObstacleDetector(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_detector')
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.marker_pub = self.create_publisher(
            MarkerArray, '/dynamic_obstacles', 10)
        
        # История сканов для обнаружения изменений
        self.scan_history = deque(maxlen=5)
        self.previous_scan = None
        
        self.timer = self.create_timer(0.1, self.detect_dynamic)
        
        self.get_logger().info('Dynamic obstacle detector started')
    
    def scan_callback(self, msg):
        self.scan_history.append(np.array(msg.ranges))
        self.current_scan_msg = msg
    
    def detect_dynamic(self):
        """Обнаружение движущихся объектов"""
        if len(self.scan_history) < 2:
            return
        
        current_scan = self.scan_history[-1]
        previous_scan = self.scan_history[-2]
        
        # Вычисление разницы между сканами
        diff = np.abs(current_scan - previous_scan)
        
        # Порог для обнаружения движения
        motion_threshold = 0.3  # метров
        
        # Поиск точек с значительным изменением
        moving_points = np.where(
            (diff > motion_threshold) & 
            (current_scan < 5.0) & 
            (previous_scan < 5.0)
        )[0]
        
        if len(moving_points) > 0:
            self.publish_markers(moving_points, current_scan)
    
    def publish_markers(self, indices, scan):
        """Публикация маркеров динамических препятствий"""
        marker_array = MarkerArray()
        
        for idx in indices:
            angle = self.current_scan_msg.angle_min + \
                    idx * self.current_scan_msg.angle_increment
            distance = scan[idx]
            
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'dynamic_obstacles'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.3
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5 секунды
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = DynamicObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
