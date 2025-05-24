import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, Quaternion
import board
import busio
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Upewnij się, że biblioteka adafruit_bno08x jest zainstalowana
# pip install adafruit-circuitpython-bno08x
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.get_logger().info('Węzeł imu_publisher został uruchomiony.')

        # --- Inicjalizacja I2C i czujnika BNO08x ---
        self.get_logger().info('Inicjalizacja magistrali I2C i czujnika BNO08x...')
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c)
            self.get_logger().info('Czujnik BNO08x zainicjalizowany pomyślnie.')
        except Exception as e:
            self.get_logger().error(f'Nie można zainicjalizować czujnika BNO08x: {e}')
            self.get_logger().error('Upewnij się, że czujnik jest poprawnie podłączony i adres I2C jest prawidłowy.')
            # Zatrzymujemy działanie, jeśli nie ma czujnika
            rclpy.shutdown()
            return
        
        time.sleep(1) # Czas na stabilizację czujnika

        # --- Włączanie odczytów z czujnika ---
        self.get_logger().info('Włączanie funkcji czujnika (akcelerometr, żyroskop, magnetometr, wektor rotacji)...')
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        self.get_logger().info('Funkcje czujnika zostały włączone.')

        # --- Konfiguracja QoS ---
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Bufor na 10 wiadomości
        )

        # --- Tworzenie publisherów ---
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', sensor_qos_profile)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', sensor_qos_profile)
        self.get_logger().info(f"Publikowanie danych IMU na temacie: '{self.imu_pub.topic_name}'")
        self.get_logger().info(f"Publikowanie danych magnetometru na temacie: '{self.mag_pub.topic_name}'")

        # --- Tworzenie timera ---
        timer_period = 0.01  # sekundy (dla częstotliwości 10 Hz)
        self.timer = self.create_timer(timer_period, self.publish_data)
        self.get_logger().info(f'Utworzono timer do publikacji danych co {timer_period} s ({1/timer_period} Hz).')
        self.get_logger().info('Węzeł jest gotowy i rozpoczął publikowanie danych.')


    def publish_data(self):
        # --- Odczyt danych z czujnika ---
        accel = self.bno.acceleration
        gyro = self.bno.gyro
        quat = self.bno.quaternion
        mag = self.bno.magnetic

        # Sprawdzenie, czy wszystkie dane są dostępne
        if None in (accel, gyro, quat, mag):
            self.get_logger().warn('Brak kompletu danych z IMU, pomijam publikację.')
            return

        # --- Tworzenie wiadomości IMU ---
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # --- Tworzenie wiadomości magnetometru ---
        mag_msg = MagneticField()
        mag_msg.header = imu_msg.header # Używamy tego samego stempla czasowego i ramki
        mag_msg.magnetic_field.x = mag[0]
        mag_msg.magnetic_field.y = mag[1]
        mag_msg.magnetic_field.z = mag[2]

        # --- Publikacja wiadomości ---
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
        
        # Log informacyjny o publikacji danych (można go zakomentować, jeśli generuje zbyt dużo logów)
        # self.get_logger().info('Opublikowano dane z IMU i magnetometru.')
        
        # Bardziej szczegółowy log (używaj z poziomem DEBUG, aby nie zaśmiecać konsoli)
        self.get_logger().debug(f'Opublikowano: Quat W:{quat[3]:.2f}, Accel X:{accel[0]:.2f}, Gyro X:{gyro[0]:.2f}, Mag X:{mag[0]:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    # Utworzenie węzła w bloku try-except, aby poprawnie obsłużyć błędy inicjalizacji
    try:
        node = IMUPublisher()
        if rclpy.ok(): # Sprawdzenie, czy węzeł został poprawnie zainicjalizowany
            rclpy.spin(node)
    except Exception as e:
        # Logowanie błędu, jeśli wystąpił podczas tworzenia węzła
        # W tym przypadku logger węzła nie jest dostępny, więc używamy ogólnego
        rclpy.logging.get_logger('main').error(f'Wystąpił błąd krytyczny: {e}')
    finally:
        # Ten blok wykona się zawsze, zapewniając czyste zamknięcie
        if 'node' in locals() and rclpy.ok():
            node.get_logger().info('Zamykanie węzła imu_publisher.')
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()