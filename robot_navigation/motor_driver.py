#!/usr/bin/env python3
"""
Драйвер моторов с реальной одометрией и TF
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import subprocess
import math
import time


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Параметры
        self.declare_parameter('left_forward_pin', 17)
        self.declare_parameter('left_backward_pin', 27)
        self.declare_parameter('left_pwm_pin', 12)
        self.declare_parameter('right_forward_pin', 22)
        self.declare_parameter('right_backward_pin', 23)
        self.declare_parameter('right_pwm_pin', 13)
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('max_pwm', 1023)
        self.declare_parameter('max_speed', 0.5)

        self.pins = {
            'lf': self.get_parameter('left_forward_pin').value,
            'lb': self.get_parameter('left_backward_pin').value,
            'lp': self.get_parameter('left_pwm_pin').value,
            'rf': self.get_parameter('right_forward_pin').value,
            'rb': self.get_parameter('right_backward_pin').value,
            'rp': self.get_parameter('right_pwm_pin').value,
        }

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_speed = self.get_parameter('max_speed').value

        # Инициализация GPIO
        self._init_gpio()

        # Подписка на cmd_vel
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Состояние одометрии
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Текущие скорости
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Таймер публикации одометрии (20 Hz)
        self.odom_timer = self.create_timer(0.05, self.publish_odom)

        # Watchdog
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.safety_watchdog)
        self.watchdog_triggered = False

        self.get_logger().info('✅ Motor Driver готов')

    def _init_gpio(self):
        try:
            subprocess.run(['gpio', '-g', 'pwm-ms'], check=True)
            subprocess.run(['gpio', '-g', 'pwmr', '1024'], check=True)
            subprocess.run(['gpio', '-g', 'pwmc', '32'], check=True)
            for name, pin in self.pins.items():
                if 'p' in name:
                    subprocess.run(['gpio', '-g', 'mode', str(pin), 'pwm'], check=True)
                else:
                    subprocess.run(['gpio', '-g', 'mode', str(pin), 'out'], check=True)
            self.get_logger().info('GPIO инициализирован')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'GPIO init failed: {e}')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z

        v_left = self.current_linear - (self.current_angular * self.wheel_base / 2.0)
        v_right = self.current_linear + (self.current_angular * self.wheel_base / 2.0)

        v_left = max(-self.max_speed, min(self.max_speed, v_left))
        v_right = max(-self.max_speed, min(self.max_speed, v_right))

        pwm_left = int((v_left / self.max_speed) * self.max_pwm)
        pwm_right = int((v_right / self.max_speed) * self.max_pwm)

        self.set_motor('left', v_left, pwm_left)
        self.set_motor('right', v_right, pwm_right)

    def set_motor(self, side: str, speed: float, pwm: int):
        abs_pwm = abs(pwm)
        if side == 'left':
            fwd_pin, bwd_pin, pwm_pin = self.pins['lf'], self.pins['lb'], self.pins['lp']
        else:
            fwd_pin, bwd_pin, pwm_pin = self.pins['rf'], self.pins['rb'], self.pins['rp']

        if speed > 0.01:
            subprocess.run(['gpio', '-g', 'write', str(fwd_pin), '1'])
            subprocess.run(['gpio', '-g', 'write', str(bwd_pin), '0'])
        elif speed < -0.01:
            subprocess.run(['gpio', '-g', 'write', str(fwd_pin), '0'])
            subprocess.run(['gpio', '-g', 'write', str(bwd_pin), '1'])
        else:
            subprocess.run(['gpio', '-g', 'write', str(fwd_pin), '0'])
            subprocess.run(['gpio', '-g', 'write', str(bwd_pin), '0'])

        subprocess.run(['gpio', '-g', 'pwm', str(pwm_pin), str(abs_pwm)])

    def publish_odom(self):
        """Интегрирование одометрии и публикация TF"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Интегрирование (простая модель без энкодеров)
        self.x += self.current_linear * math.cos(self.theta) * dt
        self.y += self.current_linear * math.sin(self.theta) * dt
        self.theta += self.current_angular * dt

        # Кватернион
        q_z = math.sin(self.theta / 2)
        q_w = math.cos(self.theta / 2)

        # Публикация TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        # Публикация Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.angular.z = self.current_angular
        self.odom_pub.publish(odom)

    def safety_watchdog(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > 0.5:
            if not self.watchdog_triggered:
                self.get_logger().warn('⚠️ Watchdog: остановка моторов')
                self.watchdog_triggered = True
            self.set_motor('left', 0.0, 0)
            self.set_motor('right', 0.0, 0)
            self.current_linear = 0.0
            self.current_angular = 0.0

    def destroy_node(self):
        self.set_motor('left', 0.0, 0)
        self.set_motor('right', 0.0, 0)
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
