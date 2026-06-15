#!/usr/bin/env python3
"""
Автоматический тест-раннер 
Запуск: python3 robot_navigation/run_all_tests.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
import time, os

RESULTS = {}

class TestRunner(Node):
    def __init__(self):
        super().__init__('test_runner')
        self.create_subscription(LaserScan, '/scan', lambda m: self.recv('/scan'), 1)
        self.create_subscription(Imu, '/imu/data', lambda m: self.recv('/imu/data'), 1)
        self.create_subscription(Odometry, '/odom', lambda m: self.recv('/odom'), 1)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.recv('/ekf'), 1)
        self.create_subscription(DiagnosticArray, '/diagnostics', lambda m: self.recv('/diag'), 1)
        
        self.timer = self.create_timer(5.0, self.evaluate)

    def recv(self, topic): RESULTS[topic] = True

    def evaluate(self):
        print("\n" + "="*60)
        print("📊 АВТОМАТИЧЕСКИЙ ОТЧЁТ ТЕСТИРОВАНИЯ")
        print("="*60)
        tests = {
            '🟢 Уровень 3': ['/scan', '/imu/data', '/odom'],
            '🟡 Уровень 4': ['/ekf', '/diag'],
            '🔴 Уровень 5': ['/scan'] # Базовая проверка стабильности
        }
        level_scores = {'3': 0, '4': 0, '5': 0}
        
        with open('test_report.txt', 'w', encoding='utf-8') as f:
            for level, topics in tests.items():
                print(f"\n{level}")
                f.write(f"{level}\n")
                for t in topics:
                    status = "✅" if RESULTS.get(t, False) else "❌"
                    print(f"  {status} {t:20}")
                    f.write(f"  {status} {t}\n")
                    if status == "✅": level_scores[level[0]] += 1

        print("\n🎓 ИТОГ:")
        print(f"  Уровень 3: {level_scores['3']}/3 {'✅ ЗАЧЁТ' if level_scores['3']==3 else '❌'}")
        print(f"  Уровень 4: {level_scores['4']}/2 {'✅ ЗАЧЁТ' if level_scores['4']==2 else '❌'}")
        print(f"  Уровень 5: Базовая стабильность {'✅' if level_scores['5']>0 else '❌'}")
        print(f"\n📄 Полный отчёт сохранён в: test_report.txt")
        print("="*60)
        rclpy.shutdown()

def main():
    rclpy.init()
    print("⏳ Сбор данных в течение 5 секунд...")
    print("💡 Убедитесь, что запущены: basic_robot_launch.py, ekf_launch.py, system_monitor.py")
    node = TestRunner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass

if __name__ == '__main__':
    main()
