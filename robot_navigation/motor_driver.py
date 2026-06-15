import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess

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
        self.get_logger().info('Initializing GPIO...')
        try:
            subprocess.run(['gpio', '-g', 'pwm-ms'], check=True)
            subprocess.run(['gpio', '-g', 'pwmr', '1024'], check=True)
            subprocess.run(['gpio', '-g', 'pwmc', '32'], check=True)
            for name, pin in self.pins.items():
                if 'p' in name:
                    subprocess.run(['gpio', '-g', 'mode', str(pin), 'pwm'], check=True)
                else:
                    subprocess.run(['gpio', '-g', 'mode', str(pin), 'out'], check=True)
            self.get_logger().info('GPIO initialized successfully.')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'GPIO init failed: {e}')

        # Подписка на cmd_vel
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Watchdog
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.1, self.safety_watchdog)
        self.watchdog_timeout = 0.5
        self.watchdog_triggered = False

        # Публикация одометрии (для уровня 3 — заглушка)
        from nav_msgs.msg import Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_timer = self.create_timer(0.05, self.publish_dummy_odom)

        self.get_logger().info('Motor Driver Node is ready.')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        if self.watchdog_triggered:
            self.get_logger().info('Connection restored.')
            self.watchdog_triggered = False

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Дифференциальный привод
        v_left = linear_x - (angular_z * self.wheel_base / 2.0)
        v_right = linear_x + (angular_z * self.wheel_base / 2.0)

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

        # Направление
        if speed > 0.01:
            subprocess.run(['gpio', '-g', 'write', str(fwd_pin), '1'])
            subprocess.run(['gpio', '-g', 'write', str(bwd_pin), '0'])
        elif speed < -0.01:
            subprocess.run(['gpio', '-g', 'write', str(fwd_pin), '0'])
            subprocess.run(['gpio', '-g', 'write', str(bwd_pin), '1'])
        else:
            subprocess.run(['gpio', '-g', 'write', str(fwd_pin), '0'])
            subprocess.run(['gpio', '-g', 'write', str(bwd_pin), '0'])

        # !!! ИСПРАВЛЕНИЕ: отправляем реальный PWM, а не 0 !!!
        subprocess.run(['gpio', '-g', 'pwm', str(pwm_pin), str(abs_pwm)])

    def publish_dummy_odom(self):
        """Заглушка одометрии для уровня 3"""
        from nav_msgs.msg import Odometry
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.odom_pub.publish(msg)

    def safety_watchdog(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.watchdog_timeout:
            if not self.watchdog_triggered:
                self.get_logger().warn('Watchdog: stopping motors.')
                self.watchdog_triggered = True
            self.stop_motors()

    def stop_motors(self):
        self.set_motor('left', 0.0, 0)
        self.set_motor('right', 0.0, 0)

    def destroy_node(self):
        self.get_logger().info('Shutting down. Stopping motors...')
        self.stop_motors()
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
