#!/usr/bin/env python3
"""
Драйвер моторов для 4-колесной платформы (M3 отключен).
Включает простые переключатели для инверсии направления, если робот едет не туда.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import subprocess
import math

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # =====================================================================
        # 🛠️ НАСТРОЙКИ НАПРАВЛЕНИЯ (ИСПРАВЛЕНИЕ ПЕРЕПУТАННЫХ НАПРАВЛЕНИЙ)
        # =====================================================================
        # Если при нажатии 'w' робот едет НАЗАД, а при 'x' ВПЕРЕД -> поставьте -1.0
        # Если при нажатии 'w' робот едет ВПЕРЕД правильно -> оставьте 1.0
        self.INVERT_LINEAR = - 1.0  # Поменяйте на -1.0, если вперед/назад перепутаны

        # Если при нажатии 'a' робот поворачивает ВПРАВО, а при 'd' ВЛЕВО -> поставьте -1.0
        # Если повороты работают правильно -> оставьте 1.0
        self.INVERT_ANGULAR = - 1.0 # Поменяйте на -1.0, если лево/право перепутаны
        # =====================================================================

        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_pwm', 1023)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value

        # Конфигурация GPIO строго по вашему описанию
        # ВАЖНО: Если инвертируется только ОДНА сторона (например, левая едет назад, а правая вперед),
        # поменяйте местами значения dir1 и dir2 для этого конкретного мотора.
        self.motors = {
            'M1': {'dir1': 20, 'dir2': 21, 'pwm': 0},   # Левое переднее
            'M4': {'dir1': 22, 'dir2': 23, 'pwm': 1},   # Правое переднее
            'M2': {'dir1': 26, 'dir2': 27, 'pwm': 13},  # Левое заднее (помощь в повороте)
            # M3 (24, 25, 12) намеренно исключен
        }

        self._init_gpio()

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.current_v = 0.0
        self.current_w = 0.0

        self.odom_timer = self.create_timer(0.05, self.publish_odom)
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.safety_watchdog)
        self.stopped = True

        self.get_logger().info('✅ Драйвер моторов запущен. Проверьте направление движения.')

    def _init_gpio(self):
        self.get_logger().info('Инициализация GPIO...')
        try:
            subprocess.run(['gpio', '-g', 'pwm-ms'], check=True, capture_output=True)
            subprocess.run(['gpio', '-g', 'pwmr', '1024'], check=True, capture_output=True)
            subprocess.run(['gpio', '-g', 'pwmc', '32'], check=True, capture_output=True)
            
            for name, pins in self.motors.items():
                subprocess.run(['gpio', '-g', 'mode', str(pins['pwm']), 'pwm'], check=True, capture_output=True)
                subprocess.run(['gpio', '-g', 'mode', str(pins['dir1']), 'out'], check=True, capture_output=True)
                subprocess.run(['gpio', '-g', 'mode', str(pins['dir2']), 'out'], check=True, capture_output=True)
            self.get_logger().info('✅ GPIO настроен успешно')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'❌ Ошибка инициализации GPIO: {e}')

    def _set_wheel(self, motor_name, speed):
        """Устанавливает скорость и направление для конкретного мотора"""
        if motor_name not in self.motors:
            return
        
        pins = self.motors[motor_name]
        pwm_val = int(abs(speed) / self.max_speed * self.max_pwm)
        pwm_val = max(0, min(1023, pwm_val))

        # Логика направления: если speed > 0, включаем dir1. Если < 0, включаем dir2.
        if speed > 0.01:
            subprocess.run(['gpio', '-g', 'write', str(pins['dir1']), '1'], capture_output=True)
            subprocess.run(['gpio', '-g', 'write', str(pins['dir2']), '0'], capture_output=True)
        elif speed < -0.01:
            subprocess.run(['gpio', '-g', 'write', str(pins['dir1']), '0'], capture_output=True)
            subprocess.run(['gpio', '-g', 'write', str(pins['dir2']), '1'], capture_output=True)
        else:
            subprocess.run(['gpio', '-g', 'write', str(pins['dir1']), '0'], capture_output=True)
            subprocess.run(['gpio', '-g', 'write', str(pins['dir2']), '0'], capture_output=True)

        subprocess.run(['gpio', '-g', 'pwm', str(pins['pwm']), str(pwm_val)], capture_output=True)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.stopped = False
        
        # 🔄 ПРИМЕНЯЕМ КОЭФФИЦИЕНТЫ ИНВЕРСИИ ЗДЕСЬ
        v = msg.linear.x * self.INVERT_LINEAR
        w = msg.angular.z * self.INVERT_ANGULAR
        
        self.current_v = v
        self.current_w = w

        # Кинематика: M1 и M4 - основная тяга, M2 - помощь в повороте
        speed_m1 = v - (w * (self.wheel_base / 2.0))
        speed_m4 = v + (w * (self.wheel_base / 2.0))
        
        # M2 работает только на поворот (помогает задней части развернуться)
        speed_m2 = w * (self.wheel_base / 2.0)

        # Ограничение скоростей
        speed_m1 = max(-self.max_speed, min(self.max_speed, speed_m1))
        speed_m4 = max(-self.max_speed, min(self.max_speed, speed_m4))
        speed_m2 = max(-self.max_speed, min(self.max_speed, speed_m2))

        # Применение к моторам
        self._set_wheel('M1', speed_m1)
        self._set_wheel('M4', speed_m4)
        self._set_wheel('M2', speed_m2)

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.x += self.current_v * math.cos(self.theta) * dt
        self.y += self.current_v * math.sin(self.theta) * dt
        self.theta += self.current_w * dt

        q_z = math.sin(self.theta / 2)
        q_w = math.cos(self.theta / 2)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = self.current_v
        odom.twist.twist.angular.z = self.current_w
        self.odom_pub.publish(odom)

    def safety_watchdog(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            if not self.stopped:
                self.get_logger().warn('⚠️ Watchdog: остановка моторов')
                self.stopped = True
                self._set_wheel('M1', 0.0)
                self._set_wheel('M4', 0.0)
                self._set_wheel('M2', 0.0)
                self.current_v = 0.0
                self.current_w = 0.0

    def destroy_node(self):
        self._set_wheel('M1', 0.0)
        self._set_wheel('M4', 0.0)
        self._set_wheel('M2', 0.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
