#!/usr/bin/env python3
"""
IMU Driver для Waveshare 10 DOF IMU (D) с чипом ICM20948
Поддерживает: акселерометр, гироскоп, магнитометр (полная ориентация)
Частота публикации: 50 Hz
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
import board
import busio
import time
import math
import numpy as np
from tf_transformations import quaternion_from_euler

# Импорт библиотек ICM20948
try:
    import adafruit_icm20x
    HAS_ICM20X = True
except ImportError:
    HAS_ICM20X = False
    print("❌ Установите: pip3 install adafruit-circuitpython-icm20x")


class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        # Параметры
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('i2c_address', 0x69)
        self.declare_parameter('calibrate_on_start', True)
        self.declare_parameter('calibration_samples', 200)

        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        i2c_address = self.get_parameter('i2c_address').value
        calibrate = self.get_parameter('calibrate_on_start').value
        cal_samples = self.get_parameter('calibration_samples').value

        # Инициализация I2C
        self.get_logger().info('Инициализация I2C...')
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Инициализация ICM20948
        self.icm = None
        self._init_icm20948(i2c_address)

        if self.icm is None:
            raise RuntimeError('ICM20948 не найден!')

        # Калибровка
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.mag_offset = np.array([0.0, 0.0, 0.0])
        self.mag_scale = np.array([1.0, 1.0, 1.0])

        if calibrate:
            self.calibrate(samples=cal_samples)

        # Публикаторы
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Комплементарный фильтр для ориентации
        self.dt = 1.0 / publish_rate
        self.alpha = 0.96  # Коэффициент фильтра
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw

        # Таймер публикации
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_data)

        self.get_logger().info(
            f'✅ ICM20948 инициализирован @ 0x{i2c_address:x}, '
            f'частота {publish_rate} Hz'
        )

    def _init_icm20948(self, address):
        """Попытка инициализации ICM20948"""
        # Попытка по указанному адресу
        for addr in [address, 0x69, 0x68]:
            try:
                self.get_logger().info(f'Попытка ICM20948 @ 0x{addr:x}...')
                self.icm = adafruit_icm20x.ICM20948(self.i2c, address=addr)
                self.get_logger().info(f'✅ ICM20948 найден @ 0x{addr:x}')

                # Настройка диапазонов
                # У ICM20948 в adafruit библиотеке диапазоны фиксированные:
                # Accelerometer: ±4g
                # Gyroscope: ±500 dps
                return
            except Exception as e:
                self.get_logger().warn(f'Не удалось @ 0x{addr:x}: {e}')
                continue

        self.get_logger().error('❌ ICM20948 не найден ни по одному адресу!')

    def calibrate(self, samples=200):
        """Калибровка IMU в состоянии покоя"""
        self.get_logger().info(
            f'🔧 Калибровка ({samples} образцов)... '
            f'Держите датчик НЕПОДВИЖНО!'
        )

        accel_samples = []
        gyro_samples = []
        mag_samples = []

        for i in range(samples):
            try:
                accel = self.icm.acceleration
                gyro = self.icm.gyro
                mag = self.icm.magnetic

                accel_samples.append(accel)
                gyro_samples.append(gyro)
                mag_samples.append(mag)
            except Exception as e:
                self.get_logger().warn(f'Ошибка чтения: {e}')
                continue

            time.sleep(0.01)

            if (i + 1) % 50 == 0:
                self.get_logger().info(f'  Прогресс: {i + 1}/{samples}')

        if len(accel_samples) < 10:
            self.get_logger().error('Недостаточно данных для калибровки!')
            return

        # Вычисление смещений
        accel_arr = np.array(accel_samples)
        gyro_arr = np.array(gyro_samples)
        mag_arr = np.array(mag_samples)

        # Гироскоп: среднее = смещение
        self.gyro_offset = np.mean(gyro_arr, axis=0)

        # Акселерометр: среднее, минус гравитация по Z
        self.accel_offset = np.mean(accel_arr, axis=0)
        self.accel_offset[2] -= 9.80665

        # Магнитометр: hard-iron (смещение центра)
        self.mag_offset = np.mean(mag_arr, axis=0)

        # Soft-iron (масштабирование) — упрощённая версия
        mag_max = np.max(mag_arr, axis=0)
        mag_min = np.min(mag_arr, axis=0)
        mag_avg_range = (mag_max - mag_min) / 2.0
        avg_range = np.mean(mag_avg_range)
        self.mag_scale = avg_range / np.where(mag_avg_range > 0, mag_avg_range, 1.0)

        self.get_logger().info('✅ Калибровка завершена!')
        self.get_logger().info(
            f'  Accel offset: {self.accel_offset.round(4)}'
        )
        self.get_logger().info(
            f'  Gyro offset:  {self.gyro_offset.round(4)}'
        )
        self.get_logger().info(
            f'  Mag offset:   {self.mag_offset.round(2)}'
        )

    def _compute_orientation(self, accel, mag):
        """
        Вычисление ориентации (roll, pitch, yaw)
        используя акселерометр + магнитометр
        """
        ax, ay, az = accel
        mx, my, mz = mag

        # Roll и Pitch из акселерометра
        roll = math.atan2(ay, math.sqrt(ax * ax + az * az))
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Компенсация наклона для магнитометра
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)

        mx_comp = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch
        my_comp = my * cos_roll - mz * sin_roll

        # Yaw (heading) из магнитометра
        yaw = math.atan2(-my_comp, mx_comp)

        return roll, pitch, yaw

    def _apply_complementary_filter(self, roll, pitch, yaw, gyro):
        """Комплементарный фильтр: гироскоп + акселерометр/магнитометр"""
        # Интегрирование гироскопа
        gyro_roll = self.orientation[0] + gyro[0] * self.dt
        gyro_pitch = self.orientation[1] + gyro[1] * self.dt
        gyro_yaw = self.orientation[2] + gyro[2] * self.dt

        # Фильтрация
        self.orientation[0] = self.alpha * gyro_roll + (1 - self.alpha) * roll
        self.orientation[1] = self.alpha * gyro_pitch + (1 - self.alpha) * pitch
        self.orientation[2] = self.alpha * gyro_yaw + (1 - self.alpha) * yaw

        return self.orientation

    def publish_data(self):
        """Публикация данных IMU"""
        try:
            # Сырые данные
            accel_raw = np.array(self.icm.acceleration)
            gyro_raw = np.array(self.icm.gyro)
            mag_raw = np.array(self.icm.magnetic)

            # Применение калибровки
            accel = accel_raw - self.accel_offset
            gyro = gyro_raw - self.gyro_offset
            mag = (mag_raw - self.mag_offset) * self.mag_scale

            # Вычисление ориентации
            roll, pitch, yaw = self._compute_orientation(accel, mag)
            roll_f, pitch_f, yaw_f = self._apply_complementary_filter(
                roll, pitch, yaw, gyro
            )

            # Преобразование в кватернион
            q = quaternion_from_euler(roll_f, pitch_f, yaw_f)

            # === Сообщение Imu ===
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Ориентация
            imu_msg.orientation = Quaternion(
                x=float(q[0]), y=float(q[1]),
                z=float(q[2]), w=float(q[3])
            )
            # Ковариация ориентации (настроена)
            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.02
            ]

            # Угловая скорость (рад/с)
            imu_msg.angular_velocity = Vector3(
                x=float(gyro[0]),
                y=float(gyro[1]),
                z=float(gyro[2])
            )
            imu_msg.angular_velocity_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001
            ]

            # Линейное ускорение (м/с²)
            imu_msg.linear_acceleration = Vector3(
                x=float(accel[0]),
                y=float(accel[1]),
                z=float(accel[2])
            )
            imu_msg.linear_acceleration_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            self.imu_pub.publish(imu_msg)

            # === Сообщение MagneticField ===
            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            # ICM20948 возвращает uT, MagneticField ожидает Tesla
            mag_msg.magnetic_field = Vector3(
                x=float(mag[0]) * 1e-6,
                y=float(mag[1]) * 1e-6,
                z=float(mag[2]) * 1e-6
            )
            mag_msg.magnetic_field_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.05
            ]
            self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().error(f'Ошибка публикации: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IMUDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ Критическая ошибка: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
