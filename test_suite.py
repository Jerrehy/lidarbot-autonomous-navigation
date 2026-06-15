#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, MapMetaData
import time

class TestSuite(Node):
    def __init__(self):
        super().__init__('test_suite')
        self.received = {'/scan': False, '/imu/data': False, '/odom': False, '/map': False}
        
        self.create_subscription(LaserScan, '/scan', lambda m: self.set_true('/scan'), 1)
        self.create_subscription(Imu, '/imu/data', lambda m: self.set_true('/imu/data'), 1)
        self.create_subscription(Odometry, '/odom', lambda m: self.set_true('/odom'), 1)
        self.create_subscription(MapMetaData, '/map_metadata', lambda m: self.set_true('/map'), 1)
        
        self.timer = self.create_timer(3.0, self.finish_test) # Ждём 3 секунды

    def set_true(self, topic):
        self.received[topic] = True

    def finish_test(self):
        print("\n" + "="*50)
        print("📊 РЕЗУЛЬТАТЫ ТЕСТИРОВАНИЯ (Уровень 3)")
        print("="*50)
        passed = 0
        for topic, status in self.received.items():
            icon = "✅" if status else "❌"
            print(f"{icon} {topic:15} : {'Данные получены' if status else 'НЕТ ДАННЫХ'}")
            if status: passed += 1
            
        print("="*50)
        if passed >= 3:
            print("🎉 ОЦЕНКА 3: ПОДТВЕРЖДЕНА")
        else:
            print("⚠️ ОЦЕНКА 3: НЕ ПОДТВЕРЖДЕНА (проверьте запуск нод)")
        print("="*50)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = TestSuite()
    print("⏳ Сбор данных в течение 3 секунд...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
