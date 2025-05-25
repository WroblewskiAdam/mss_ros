import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime

from sensor_msgs.msg import Imu
from gps_rtk_msgs.msg import GpsRtk
from std_msgs.msg import Int32  # Import wiadomości dla serwa
import message_filters  # Import dla synchronizacji
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from my_robot_interfaces.msg import StampedInt32



class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger_node_sync')

        # --- Konfiguracja plików wyjściowych ---
        log_directory = os.path.join(os.path.expanduser('~'), 'ros2_logs_raw')
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Plik dla zsynchronizowanych danych GPS i serwa
        self.sync_log_path = os.path.join(log_directory, f'gps_servo_log_{timestamp}.csv')
        self.sync_csv_file = open(self.sync_log_path, 'w', newline='')
        self.sync_csv_writer = csv.writer(self.sync_csv_file)
        # Dodajemy kolumny dla serwa
        self.sync_csv_writer.writerow(['ros_time_sec', 'ros_time_nsec', 'rtk_status', 'latitude_deg', 'longitude_deg', 'altitude_m', 'speed_mps', 'heading_deg', 'servo_target_angle', 'servo_current_position'])
        self.get_logger().info(f"Logowanie zsynchronizowanych danych GPS i serwa do: {self.sync_log_path}")

        # Plik dla IMU (pozostaje bez zmian)
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

        # --- Subskrypcje z synchronizacją ---
        self.gps_subscription = message_filters.Subscriber(
            self,
            GpsRtk,
            '/gps_rtk_data',
            qos_profile=sensor_qos_profile)
        
        self.servo_target_subscription = message_filters.Subscriber(
            self,
            StampedInt32,
            '/servo/set_angle',
            qos_profile=sensor_qos_profile)

        self.servo_position_subscription = message_filters.Subscriber(
            self,
            StampedInt32,
            '/servo/position',
            qos_profile=sensor_qos_profile)
        
        # Synchronizator - ok. 10ms tolerancji na różnicę w stemplach czasowych
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.gps_subscription, self.servo_target_subscription, self.servo_position_subscription],
            queue_size=10,
            slop=0.01)
        
        self.time_synchronizer.registerCallback(self.sync_callback)

        # Subskrypcja dla IMU (bez synchronizacji)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            qos_profile=sensor_qos_profile)

    def sync_callback(self, gps_msg, servo_target_msg, servo_position_msg):
        """Callback dla zsynchronizowanych danych."""
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
            servo_target_msg.data,
            servo_position_msg.data
        ])
        self.sync_csv_file.flush()

    def imu_callback(self, msg):
        timestamp = msg.header.stamp
        self.imu_csv_writer.writerow([timestamp.sec, timestamp.nanosec, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.imu_csv_file.flush()

    def destroy_node(self):
        self.get_logger().info('Zamykanie plików logów.')
        if self.sync_csv_file: self.sync_csv_file.close()
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