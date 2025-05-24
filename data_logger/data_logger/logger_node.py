import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime

from sensor_msgs.msg import Imu
from gps_rtk_msgs.msg import GpsRtk
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node_no_sync')

        # --- Konfiguracja plików wyjściowych ---
        log_directory = os.path.join(os.path.expanduser('~'), 'ros2_logs_raw')
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Plik dla GPS
        self.gps_log_path = os.path.join(log_directory, f'gps_log_{timestamp}.csv')
        self.gps_csv_file = open(self.gps_log_path, 'w', newline='')
        self.gps_csv_writer = csv.writer(self.gps_csv_file)
        self.gps_csv_writer.writerow(['ros_time_sec', 'ros_time_nsec', 'rtk_status', 'latitude_deg', 'longitude_deg', 'altitude_m', 'speed_mps', 'heading_deg'])
        self.get_logger().info(f"Logowanie danych GPS do: {self.gps_log_path}")

        # Plik dla IMU
        self.imu_log_path = os.path.join(log_directory, f'imu_log_{timestamp}.csv')
        self.imu_csv_file = open(self.imu_log_path, 'w', newline='')
        self.imu_csv_writer = csv.writer(self.imu_csv_file)
        self.imu_csv_writer.writerow(['ros_time_sec', 'ros_time_nsec', 'linear_accel_x', 'linear_accel_y', 'linear_accel_z', 'angular_velo_x', 'angular_velo_y', 'angular_velo_z'])
        self.get_logger().info(f"Logowanie danych IMU do: {self.imu_log_path}")

        # --- QoS Profile dla subskrypcji ---
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 
        )

        # --- Subskrypcje (bez synchronizacji) ---
        self.gps_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data',
            self.gps_callback,
            qos_profile=sensor_qos_profile)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            qos_profile=sensor_qos_profile)

    def gps_callback(self, msg):
        timestamp = msg.header.stamp
        self.gps_csv_writer.writerow([timestamp.sec, timestamp.nanosec, msg.rtk_status, msg.latitude_deg, msg.longitude_deg, msg.altitude_m, msg.speed_mps, msg.heading_deg])
        self.gps_csv_file.flush()

    def imu_callback(self, msg):
        timestamp = msg.header.stamp
        self.imu_csv_writer.writerow([timestamp.sec, timestamp.nanosec, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.imu_csv_file.flush()

    def destroy_node(self):
        self.get_logger().info('Zamykanie plików logów.')
        if self.gps_csv_file: self.gps_csv_file.close()
        if self.imu_csv_file: self.imu_csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    data_logger_node = DataLoggerNode()
    try:
        rclpy.spin(data_logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        data_logger_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()