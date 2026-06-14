"""
Драйвер для Waveshare 10 DOF IMU
Чтение акселерометра, гироскопа, магнитометра
Вычисление ориентации и скорости
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import board
import busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lis3mdl import LIS3MDL
import math
import time

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        
        # Инициализация I2C
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Инициализация датчиков
        self.accel_gyro = LSM6DSOX(self.i2c)
        self.magnetometer = LIS3MDL(self.i2c)
        
        # Публикация IMU данных
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, 'imu/odom', 10)
        
        # Параметры для интегрирования
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion [x, y, z, w]
        
        self.last_time = time.time()
        
        # Калибровочные данные
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.accel_offset = [0.0, 0.0, 9.81]
        
        # Таймер для чтения данных
        self.timer = self.create_timer(0.02, self.read_imu)  # 50 Hz
        
        self.calibrate()
        self.get_logger().info('IMU driver initialized')
    
    def calibrate(self):
        """Калибровка гироскопа при старте"""
        self.get_logger().info('Calibrating gyroscope... Keep robot still!')
        samples = 100
        gyro_sum = [0.0, 0.0, 0.0]
        
        for _ in range(samples):
            gyro = self.accel_gyro.gyro
            gyro_sum[0] += gyro[0]
            gyro_sum[1] += gyro[1]
            gyro_sum[2] += gyro[2]
            time.sleep(0.01)
        
        self.gyro_offset = [g / samples for g in gyro_sum]
        self.get_logger().info('Calibration complete')
    
    def read_imu(self):
        """Чтение данных с IMU"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Чтение акселерометра и гироскопа
        accel = self.accel_gyro.acceleration
        gyro = self.accel_gyro.gyro
        
        # Чтение магнитометра
        mag = self.magnetometer.magnetic
        
        # Компенсация смещения гироскопа
        gyro_calibrated = [
            gyro[0] - self.gyro_offset[0],
            gyro[1] - self.gyro_offset[1],
            gyro[2] - self.gyro_offset[2]
        ]
        
        # Обновление данных
        self.linear_acceleration = list(accel)
        self.angular_velocity = list(gyro_calibrated)
        
        # Интегрирование для получения скорости (простое)
        for i in range(3):
            self.velocity[i] += self.linear_acceleration[i] * dt
        
        # Вычисление ориентации (упрощенное, без фильтра)
        self.update_orientation(gyro_calibrated, dt)
        
        # Публикация IMU сообщения
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        imu_msg.linear_acceleration.x = self.linear_acceleration[0]
        imu_msg.linear_acceleration.y = self.linear_acceleration[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2]
        
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]
        
        imu_msg.orientation.x = self.orientation[0]
        imu_msg.orientation.y = self.orientation[1]
        imu_msg.orientation.z = self.orientation[2]
        imu_msg.orientation.w = self.orientation[3]
        
        self.imu_pub.publish(imu_msg)
        
        # Публикация одометрии
        self.publish_odometry()
    
    def update_orientation(self, gyro, dt):
        """Обновление ориентации через интегрирование гироскопа"""
        # Упрощенное интегрирование (для полноценной работы нужен фильтр)
        roll = self.orientation[0] + gyro[0] * dt
        pitch = self.orientation[1] + gyro[1] * dt
        yaw = self.orientation[2] + gyro[2] * dt
        
        # Преобразование в кватернион
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        self.orientation[0] = sr * cp * cy - cr * sp * sy
        self.orientation[1] = cr * sp * cy + sr * cp * sy
        self.orientation[2] = cr * cp * sy - sr * sp * cy
        self.orientation[3] = cr * cp * cy + sr * sp * sy
    
    def publish_odometry(self):
        """Публикация одометрии на основе IMU"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Позиция (интегрирование скорости)
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]
        
        # Ориентация
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]
        odom_msg.pose.pose.orientation.w = self.orientation[3]
        
        # Скорость
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]
        
        odom_msg.twist.twist.angular.x = self.angular_velocity[0]
        odom_msg.twist.twist.angular.y = self.angular_velocity[1]
        odom_msg.twist.twist.angular.z = self.angular_velocity[2]
        
        self.odom_pub.publish(odom_msg)
        
        # Обновление позиции
        dt = 0.02
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt

def main():
    rclpy.init()
    node = IMUDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
