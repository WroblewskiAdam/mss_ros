# W pliku data_logger/data_logger/logger_node.py

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime

from sensor_msgs.msg import Imu, MagneticField # <-- DODANY MagneticField
from gps_rtk_msgs.msg import GpsRtk
from my_robot_interfaces.msg import StampedInt32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node_sync')

        # --- Konfiguracja plików ---
        log_directory = os.path.join(os.path.expanduser('~'), 'ros2_logs_raw')
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Plik dla zsynchronizowanych danych GPS i serwa (bez zmian)
        self.sync_log_path = os.path.join(log_directory, f'gps_servo_log_{timestamp}.csv')
        self.sync_csv_file = open(self.sync_log_path, 'w', newline='')
        self.sync_csv_writer = csv.writer(self.sync_csv_file)
        self.sync_csv_writer.writerow(['ros_time_sec', 'ros_time_nsec', 'rtk_status', 'latitude_deg', 'longitude_deg', 'altitude_m', 'speed_mps', 'heading_deg', 'servo_target_angle', 'servo_current_position'])
        self.get_logger().info(f"Logowanie zsynchronizowanych danych GPS i serwa do: {self.sync_log_path}")

        # Plik dla IMU (ZAKTUALIZOWANY NAGŁÓWEK)
        self.imu_log_path = os.path.join(log_directory, f'imu_log_{timestamp}.csv')
        self.imu_csv_file = open(self.imu_log_path, 'w', newline='')
        self.imu_csv_writer = csv.writer(self.imu_csv_file)
        # Dodajemy kolumny dla magnetometru
        self.imu_csv_writer.writerow(['ros_time_sec', 'ros_time_nsec', 
                                      'linear_accel_x', 'linear_accel_y', 'linear_accel_z', 
                                      'angular_velo_x', 'angular_velo_y', 'angular_velo_z',
                                      'mag_x', 'mag_y', 'mag_z']) # <-- NOWE KOLUMNY
        self.get_logger().info(f"Logowanie danych IMU (w tym magnetometru) do: {self.imu_log_path}")


        # --- Zmienne do przechowywania ostatnich wiadomości ---
        self.last_servo_target_msg = None
        self.last_servo_position_msg = None
        self.last_mag_msg = None # <-- NOWA ZMIENNA DLA MAGNETOMETRU

        # --- QoS Profile (bez zmian) ---
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 
        )

        # === Subskrypcje ===
        
        self.servo_target_subscription = self.create_subscription(
            StampedInt32,
            '/servo/set_angle',
            self.servo_target_callback,
            qos_profile=sensor_qos_profile)

        self.servo_position_subscription = self.create_subscription(
            StampedInt32,
            '/servo/position',
            self.servo_position_callback,
            qos_profile=sensor_qos_profile)

        self.gps_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data',
            self.gps_log_callback,
            qos_profile=sensor_qos_profile)

        # Subskrypcja dla /imu/data_raw (główny wyzwalacz dla logu IMU)
        self.imu_data_raw_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_log_callback, # Zmieniony callback
            qos_profile=sensor_qos_profile)
            
        # Subskrypcja dla /imu/mag (tylko do przechowywania ostatniej wartości)
        self.mag_subscription = self.create_subscription(
            MagneticField,
            '/imu/mag',
            self.mag_update_callback, # Nowy callback
            qos_profile=sensor_qos_profile)
            
        self.get_logger().info("Logger gotowy. Używa GPS jako wyzwalacza zapisu dla logu GPS/Servo, oraz /imu/data_raw dla logu IMU.")

    def servo_target_callback(self, msg):
        self.last_servo_target_msg = msg

    def servo_position_callback(self, msg):
        self.last_servo_position_msg = msg

    def gps_log_callback(self, gps_msg):
        if self.last_servo_target_msg is None or self.last_servo_position_msg is None:
            self.get_logger().warn("Otrzymano dane GPS, ale wciąż czekam na pierwsze dane z serwa. Pomijam zapis.", throttle_duration_sec=5)
            return

        timestamp = gps_msg.header.stamp
        self.sync_csv_writer.writerow([
            timestamp.sec,
            timestamp.nanosec,
            gps_msg.rtk_status,
            gps_msg.latitude_deg,
            gps_msg.longitude_deg,
            gps_msg.altitude_m,
            gps_msg.speed_mps,
            gps_msg.heading_deg,
            self.last_servo_target_msg.data,
            self.last_servo_position_msg.data
        ])
        self.sync_csv_file.flush()

    def mag_update_callback(self, msg):
        """Ten callback tylko aktualizuje ostatnią znaną wartość magnetometru."""
        self.last_mag_msg = msg

    def imu_log_callback(self, imu_msg): # Zmieniona nazwa z imu_callback na imu_log_callback
        """Główny callback dla logu IMU, wyzwalany przez /imu/data_raw."""
        # Sprawdź, czy mamy już jakiekolwiek dane z magnetometru
        if self.last_mag_msg is None:
            # Zapisz z pustymi wartościami lub ostrzeż i pomiń
            mag_x, mag_y, mag_z = float('nan'), float('nan'), float('nan') 
            self.get_logger().warn("Otrzymano dane IMU, ale wciąż czekam na pierwsze dane z magnetometru. Zapisuję IMU z wartościami NaN dla mag.", throttle_duration_sec=5)
        else:
            mag_x = self.last_mag_msg.magnetic_field.x
            mag_y = self.last_mag_msg.magnetic_field.y
            mag_z = self.last_mag_msg.magnetic_field.z
            # Opcjonalnie: sprawdź, czy stempel czasu last_mag_msg nie jest zbyt stary
            # w stosunku do imu_msg.header.stamp. Dla uproszczenia pomijamy ten krok.

        timestamp = imu_msg.header.stamp
        self.imu_csv_writer.writerow([
            timestamp.sec, 
            timestamp.nanosec, 
            imu_msg.linear_acceleration.x, 
            imu_msg.linear_acceleration.y, 
            imu_msg.linear_acceleration.z, 
            imu_msg.angular_velocity.x, 
            imu_msg.angular_velocity.y, 
            imu_msg.angular_velocity.z,
            mag_x, # <-- NOWE DANE
            mag_y, # <-- NOWE DANE
            mag_z  # <-- NOWE DANE
        ])
        self.imu_csv_file.flush()

    def destroy_node(self):
        self.get_logger().info('Zamykanie plików logów.')
        if self.sync_csv_file: self.sync_csv_file.close()
        if self.imu_csv_file: self.imu_csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    data_logger_node = None
    try:
        data_logger_node = DataLoggerNode()
        rclpy.spin(data_logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        if data_logger_node:
            data_logger_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()