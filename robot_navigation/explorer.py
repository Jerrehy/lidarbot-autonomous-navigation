"""
Алгоритм автономного исследования для построения карты
Использует frontier-based exploration
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Подписки
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Публикация команд
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Состояние
        self.current_scan = None
        self.current_map = None
        self.current_position = None
        self.current_yaw = 0.0
        
        self.exploration_complete = False
        self.visited_frontiers = []
        
        # Таймер управления
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Autonomous explorer started')
    
    def scan_callback(self, msg):
        """Обработка данных лидара"""
        self.current_scan = msg
    
    def map_callback(self, msg):
        """Обработка карты"""
        self.current_map = msg
        
        # Проверка завершения исследования
        if self.check_exploration_complete():
            self.exploration_complete = True
            self.get_logger().info('Exploration complete!')
            self.stop_robot()
    
    def odom_callback(self, msg):
        """Обработка одометрии"""
        self.current_position = msg.pose.pose.position
        # Извлечение yaw из кватерниона
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def control_loop(self):
        """Основной цикл управления"""
        if self.exploration_complete:
            self.stop_robot()
            return
        
        if self.current_scan is None:
            return
        
        # Поиск ближайшей неисследованной области (frontier)
        frontier = self.find_nearest_frontier()
        
        if frontier is None:
            # Нет доступных frontier - исследование завершено
            self.exploration_complete = True
            self.stop_robot()
            return
        
        # Движение к frontier
        self.move_to_frontier(frontier)
    
    def find_nearest_frontier(self):
        """Поиск ближайшей неисследованной области"""
        if self.current_map is None or self.current_position is None:
            return None
        
        # Преобразование карты в numpy array
        width = self.current_map.info.width
        height = self.current_map.info.height
        data = np.array(self.current_map.data).reshape((height, width))
        
        # Поиск frontier клеток (граница между свободным и неизвестным)
        frontiers = []
        for y in range(height):
            for x in range(width):
                if data[y, x] == 0:  # свободное пространство
                    # Проверка соседей на неизвестное пространство (-1)
                    has_unknown_neighbor = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < height and 0 <= nx < width:
                                if data[ny, nx] == -1:  # неизвестно
                                    has_unknown_neighbor = True
                                    break
                        if has_unknown_neighbor:
                            break
                    
                    if has_unknown_neighbor:
                        # Преобразование в координаты мира
                        world_x = (x + 0.5) * self.current_map.info.resolution + \
                                  self.current_map.info.origin.position.x
                        world_y = (y + 0.5) * self.current_map.info.resolution + \
                                  self.current_map.info.origin.position.y
                        
                        # Расстояние до робота
                        dist = math.sqrt(
                            (world_x - self.current_position.x)**2 + 
                            (world_y - self.current_position.y)**2
                        )
                        
                        frontiers.append((world_x, world_y, dist))
        
        if not frontiers:
            return None
        
        # Сортировка по расстоянию
        frontiers.sort(key=lambda f: f[2])
        
        # Возврат ближайшего frontier, которое еще не посещали
        for frontier in frontiers:
            if not self.is_visited(frontier):
                self.visited_frontiers.append(frontier)
                return frontier
        
        return frontiers[0] if frontiers else None
    
    def is_visited(self, frontier):
        """Проверка, посещали ли мы уже этот frontier"""
        threshold = 1.0  # метры
        for visited in self.visited_frontiers:
            dist = math.sqrt(
                (frontier[0] - visited[0])**2 + 
                (frontier[1] - visited[1])**2
            )
            if dist < threshold:
                return True
        return False
    
    def move_to_frontier(self, frontier):
        """Движение к цели"""
        target_x, target_y, _ = frontier
        
        # Вычисление угла к цели
        dx = target_x - self.current_position.x
        dy = target_y - self.current_position.y
        target_yaw = math.atan2(dy, dx)
        
        # Ошибка по углу
        angle_error = target_yaw - self.current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Расстояние до цели
        distance = math.sqrt(dx**2 + dy**2)
        
        # Управление
        cmd = Twist()
        
        if abs(angle_error) > 0.2:
            # Поворот на месте
            cmd.angular.z = 0.5 * angle_error / abs(angle_error)
            cmd.linear.x = 0.0
        else:
            # Движение вперед
            cmd.linear.x = min(0.3, distance * 0.5)
            cmd.angular.z = angle_error * 0.3
        
        # Проверка препятствий
        if self.obstacle_detected():
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # объезд
        
        self.cmd_pub.publish(cmd)
    
    def obstacle_detected(self):
        """Проверка наличия препятствий впереди"""
        if self.current_scan is None:
            return False
        
        # Проверка сектора впереди (±30 градусов)
        ranges = np.array(self.current_scan.ranges)
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        
        # Индексы для сектора впереди
        center_idx = int((-angle_min) / angle_increment)
        sector_range = int((math.radians(30)) / angle_increment)
        
        front_ranges = ranges[max(0, center_idx - sector_range):
                              min(len(ranges), center_idx + sector_range)]
        
        # Проверка минимального расстояния
        min_distance = min(front_ranges)
        return min_distance < 0.5  # 50 см
    
    def check_exploration_complete(self):
        """Проверка завершения исследования"""
        if self.current_map is None:
            return False
        
        data = np.array(self.current_map.data)
        unknown_ratio = np.sum(data == -1) / len(data)
        
        # Если менее 5% неизвестной области - исследование завершено
        return unknown_ratio < 0.05
    
    def stop_robot(self):
        """Остановка робота"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
