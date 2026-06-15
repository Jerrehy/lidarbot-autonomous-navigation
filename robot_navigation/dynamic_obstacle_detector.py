#!/usr/bin/env python3
"""
Детектор динамических препятствий (упрощённая надёжная версия)
Сравнивает только два последних скана лидара и находит движущиеся объекты.
Исправлено: корректно обрабатывает разную длину сканов от RPLidar.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np


class DynamicObstacleDetector(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_detector')

        # Параметры
        self.declare_parameter('threshold', 0.3)   # Мин. изменение (м) для "движения"
        self.declare_parameter('min_range', 0.15)  # Мин. дальность (фильтр шума)
        self.declare_parameter('max_range', 8.0)   # Макс. дальность
        
        self.threshold = self.get_parameter('threshold').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        # Подписки и публикации
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/dynamic_obstacles', 10)

        # Храним только ДВА последних скана (текущий и предыдущий)
        self.previous_scan = None
        self.previous_angles = None

        # Таймер публикации (5 Hz)
        self.timer = self.create_timer(0.2, self.publish_markers)

        self.get_logger().info(
            f'✅ Dynamic Obstacle Detector запущен (threshold={self.threshold}m)'
        )

    def scan_callback(self, msg: LaserScan):
        """Обработка нового скана лидара"""
        try:
            # Фильтрация невалидных значений (NaN, inf, вне диапазона)
            ranges = np.array(msg.ranges)
            valid_mask = (
                (ranges >= self.min_range) & 
                (ranges <= self.max_range) & 
                np.isfinite(ranges)
            )
            
            # Вычисляем углы (они постоянны для одного типа лидара)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

            # Применяем маску: невалидные точки заменяем на 0
            filtered_ranges = np.where(valid_mask, ranges, 0.0)
            
            # Сохраняем предыдущий скан (если он уже был)
            if self.previous_scan is not None:
                # 🔑 КЛЮЧЕВОЕ: обрезаем оба скана до минимальной длины
                min_len = min(len(filtered_ranges), len(self.previous_scan))
                self.current_scan = filtered_ranges[:min_len]
                self.current_angles = angles[:min_len]
                self.prev_trimmed = self.previous_scan[:min_len]
                self.prev_angles = self.previous_angles[:min_len]
            
            # Текущий скан становится предыдущим для следующего вызова
            self.previous_scan = filtered_ranges
            self.previous_angles = angles
            
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки скана: {e}', 
                                   throttle_duration_sec=5.0)

    def detect_dynamic(self):
        """Сравнение текущего скана с предыдущим"""
        # Проверяем, есть ли оба скана
        if not hasattr(self, 'current_scan') or self.previous_scan is None:
            return None, None, None

        # Находим различия
        diff = np.abs(self.current_scan - self.prev_trimmed)
        
        # Динамические объекты: разница > порога И обе точки валидны
        dynamic_mask = (
            (diff > self.threshold) & 
            (self.current_scan > self.min_range) & 
            (self.prev_trimmed > self.min_range)
        )

        return self.current_angles, dynamic_mask, self.current_scan

    def publish_markers(self):
        """Публикация маркеров динамических объектов в RViz2"""
        angles, dynamic_mask, ranges = self.detect_dynamic()
        
        if angles is None:
            return  # Ещё нет данных для сравнения

        marker_array = MarkerArray()
        
        # Создаём один маркер-список для всех динамических точек
        marker = Marker()
        marker.header.frame_id = 'laser'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dynamic_obstacles'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1   # Размер точки
        marker.scale.y = 0.1
        marker.color.r = 1.0   # Красный
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300_000_000  # 0.3 сек

        points = []
        for i, is_dynamic in enumerate(dynamic_mask):
            if is_dynamic:
                p = Point()
                p.x = float(ranges[i] * np.cos(angles[i]))
                p.y = float(ranges[i] * np.sin(angles[i]))
                p.z = 0.1  # Чуть выше пола для видимости
                points.append(p)

        marker.points = points
        marker_array.markers.append(marker)

        # Логируем количество обнаруженных точек (раз в 2 секунды)
        if len(points) > 0:
            self.get_logger().info(
                f'🔴 Обнаружено динамических точек: {len(points)}',
                throttle_duration_sec=2.0
            )

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
