# W pliku data_logger/data_logger/logger_node.py

import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64  # <-- NOWOŚĆ: Import dla prędkości zadanej
from my_robot_interfaces.msg import StampedInt32, GpsRtk
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

        # Plik dla IMU (bez zmian)
        self.imu_log_path = os.path.join(log_directory, f'imu_log_{timestamp}.csv')
        self.imu_csv_file = open(self.imu_log_path, 'w', newline='')
        self.imu_csv_writer = csv.writer(self.imu_csv_file)
        self.imu_csv_writer.writerow(['ros_time_sec', 'ros_time_nsec', 
                                      'linear_accel_x', 'linear_accel_y', 'linear_accel_z', 
                                      'angular_velo_x', 'angular_velo_y', 'angular_velo_z',
                                      'mag_x', 'mag_y', 'mag_z'])
        self.get_logger().info(f"Logowanie danych IMU (w tym magnetometru) do: {self.imu_log_path}")

        # --- NOWOŚĆ: Plik dla logowania danych z regulatora prędkości ---
        self.speed_control_log_path = os.path.join(log_directory, f'speed_control_log_{timestamp}.csv')
        self.speed_control_csv_file = open(self.speed_control_log_path, 'w', newline='')
        self.speed_control_csv_writer = csv.writer(self.speed_control_csv_file)
        self.speed_control_csv_writer.writerow(['ros_time_sec', 'ros_time_nsec', 'target_speed_mps', 'filtered_speed_mps', 'servo_command_angle'])
        self.get_logger().info(f"Logowanie danych regulatora prędkości do: {self.speed_control_log_path}")
        # ----------------------------------------------------------------

        # --- Zmienne do przechowywania ostatnich wiadomości ---
        self.last_servo_target_msg = None
        self.last_servo_position_msg = None
        self.last_mag_msg = None
        self.last_target_speed_msg = None # <-- NOWOŚĆ

        # --- QoS Profile (bez zmian) ---
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 
        )

        # === Subskrypcje ===
        self.servo_target_subscription = self.create_subscription(StampedInt32, '/servo/set_angle', self.servo_target_callback, qos_profile=sensor_qos_profile)
        self.servo_position_subscription = self.create_subscription(StampedInt32, '/servo/position', self.servo_position_callback, qos_profile=sensor_qos_profile)
        self.gps_subscription = self.create_subscription(GpsRtk, '/gps_rtk_data/tractor', self.gps_log_callback, qos_profile=sensor_qos_profile)
        self.imu_data_raw_subscription = self.create_subscription(Imu, '/imu/data_raw', self.imu_log_callback, qos_profile=sensor_qos_profile)
        self.mag_subscription = self.create_subscription(MagneticField, '/imu/mag', self.mag_update_callback, qos_profile=sensor_qos_profile)
            
        # --- NOWOŚĆ: Subskrypcje dla logowania regulatora prędkości ---
        # Subskrypcja dla prędkości zadanej (tylko przechowuje wartość)
        self.target_speed_subscription = self.create_subscription(
            Float64,
            '/target_speed',
            self.target_speed_callback,
            qos_profile=sensor_qos_profile)

        # Subskrypcja dla filtrowanej prędkości (główny wyzwalacz zapisu)
        self.filtered_speed_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/tractor_filtered',
            self.speed_control_log_callback, # Nowy callback do zapisu
            qos_profile=sensor_qos_profile)
        # ----------------------------------------------------------

        self.get_logger().info("Logger gotowy. Używa /gps_rtk_data_filtered jako wyzwalacza zapisu dla logu regulatora prędkości.")

    def servo_target_callback(self, msg):
        self.last_servo_target_msg = msg

    def servo_position_callback(self, msg):
        self.last_servo_position_msg = msg
        
    # --- NOWOŚĆ: Callback dla prędkości zadanej ---
    def target_speed_callback(self, msg):
        """Ten callback tylko aktualizuje ostatnią znaną wartość prędkości zadanej."""
        self.last_target_speed_msg = msg
    # -------------------------------------------

    def gps_log_callback(self, gps_msg):
        # (bez zmian)
        if self.last_servo_target_msg is None or self.last_servo_position_msg is None:
            self.get_logger().warn("Otrzymano dane GPS, ale wciąż czekam na pierwsze dane z serwa. Pomijam zapis.", throttle_duration_sec=5)
            return

        timestamp = gps_msg.header.stamp
        self.sync_csv_writer.writerow([
            timestamp.sec, timestamp.nanosec, gps_msg.rtk_status, gps_msg.latitude_deg, gps_msg.longitude_deg,
            gps_msg.altitude_m, gps_msg.speed_mps, gps_msg.heading_deg, self.last_servo_target_msg.data,
            self.last_servo_position_msg.data
        ])
        self.sync_csv_file.flush()

    def mag_update_callback(self, msg):
        # (bez zmian)
        self.last_mag_msg = msg

    def imu_log_callback(self, imu_msg): 
        # (bez zmian)
        if self.last_mag_msg is None:
            mag_x, mag_y, mag_z = float('nan'), float('nan'), float('nan') 
            self.get_logger().warn("Otrzymano dane IMU, ale wciąż czekam na pierwsze dane z magnetometru. Zapisuję IMU z wartościami NaN dla mag.", throttle_duration_sec=5)
        else:
            mag_x, mag_y, mag_z = self.last_mag_msg.magnetic_field.x, self.last_mag_msg.magnetic_field.y, self.last_mag_msg.magnetic_field.z
        
        timestamp = imu_msg.header.stamp
        self.imu_csv_writer.writerow([
            timestamp.sec, timestamp.nanosec, imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, 
            imu_msg.linear_acceleration.z, imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
            mag_x, mag_y, mag_z
        ])
        self.imu_csv_file.flush()
        
    # --- NOWOŚĆ: Główny callback do logowania danych regulatora prędkości ---
    def speed_control_log_callback(self, filtered_speed_msg):
        """Główny callback dla logu regulatora, wyzwalany przez /gps_rtk_data_filtered."""
        # Sprawdź, czy mamy już wszystkie potrzebne dane
        if self.last_target_speed_msg is None or self.last_servo_target_msg is None:
            self.get_logger().warn("Otrzymano dane prędkości z filtracji, ale brak jeszcze prędkości zadanej lub sterowania serwem. Pomijam zapis.", throttle_duration_sec=5)
            return

        # Wszystkie dane są dostępne, więc zapisujemy do pliku
        timestamp = filtered_speed_msg.header.stamp
        
        target_speed = self.last_target_speed_msg.data
        filtered_speed = filtered_speed_msg.speed_mps
        servo_angle = self.last_servo_target_msg.data
        
        self.speed_control_csv_writer.writerow([
            timestamp.sec,
            timestamp.nanosec,
            target_speed,
            filtered_speed,
            servo_angle
        ])
        self.speed_control_csv_file.flush()
    # --------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Zamykanie plików logów.')
        if self.sync_csv_file: self.sync_csv_file.close()
        if self.imu_csv_file: self.imu_csv_file.close()
        if self.speed_control_csv_file: self.speed_control_csv_file.close() # <-- NOWOŚĆ
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