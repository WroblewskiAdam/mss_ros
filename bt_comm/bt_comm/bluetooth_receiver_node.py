import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from my_robot_interfaces.msg import GpsRtk
import socket
import json
import subprocess
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from datetime import datetime, timezone # Do obsługi czasu


class BluetoothReceiverNode(Node):
    def __init__(self):
        super().__init__('bluetooth_receiver_node')

        self.declare_parameter('bt_host_mac', '2C:CF:67:A2:F3:ED')
        self.declare_parameter('bt_port', 4)
        self.declare_parameter('publish_topic', '/gps_rtk_data/chopper')

        self.host_mac = self.get_parameter('bt_host_mac').get_parameter_value().string_value
        self.port = self.get_parameter('bt_port').get_parameter_value().integer_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(GpsRtk, self.publish_topic, qos_profile)
        self.diag_publisher_ = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.server_sock.bind((self.host_mac, self.port))
        self.server_sock.listen(1)
        
        self.get_logger().info(f"Serwer Bluetooth uruchomiony na {self.host_mac} na porcie {self.port}")
        
        self.data_timer_ = self.create_timer(0.1, self.accept_and_receive_data)
        self.diag_timer_ = self.create_timer(2.0, self.publish_diagnostics)

        self.client_sock = None
        self.client_info = None

    def accept_and_receive_data(self):
        if self.client_sock is None:
            try:
                self.server_sock.settimeout(0.1) 
                self.client_sock, self.client_info = self.server_sock.accept()
                self.get_logger().info(f"Zaakceptowano połączenie od: {self.client_info}")
                self.client_sock.settimeout(1.0)
            except (socket.timeout, socket.error):
                return

        try:
            data = self.client_sock.recv(1024)
            if not data:
                self.get_logger().warn("Klient zamknął połączenie.")
                self.client_sock.close(); self.client_sock = None; self.client_info = None
                return

            try:
                gps_data = json.loads(data.decode('utf-8'))
                msg = GpsRtk()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "gps_bluetooth_link"
                
                # Uzupełnianie wszystkich pól wiadomości
                msg.latitude_deg = gps_data.get('lat', 0.0)
                msg.longitude_deg = gps_data.get('lon', 0.0)
                msg.speed_mps = gps_data.get('speed', 0.0)
                msg.heading_deg = gps_data.get('heading', 0.0)
                msg.rtk_status = gps_data.get('rtk_status', 0)
                # === NOWE POLA ===
                msg.altitude_m = gps_data.get('altitude', 0.0)
                # msg.gps_time = gps_data.get('gps_time', "")
                msg.gps_time = self._convert_gps_utc_to_ros_time(gps_data.get('gps_time', ""))

                self.publisher_.publish(msg)
            except (json.JSONDecodeError, UnicodeDecodeError):
                self.get_logger().warn(f"Otrzymano uszkodzone dane: {data}")

        except socket.error:
            self.get_logger().warn(f"Błąd połączenia Bluetooth. Klient prawdopodobnie się rozłączył.")
            if self.client_sock: self.client_sock.close()
            self.client_sock = None; self.client_info = None

    def _convert_gps_utc_to_ros_time(self, utc_time_str):
        """Konwertuje czas UTC z GNGGA (hhmmss.ss) na ROS Time (builtin_interfaces.msg.Time)"""
        if not utc_time_str or '.' not in utc_time_str:
            return self.get_clock().now().to_msg() # Zwróć aktualny czas ROS jako fallback

        try:
            # Użyj dzisiejszej daty systemowej UTC, połącz z czasem GPS
            now_utc = datetime.now(timezone.utc)
            
            hour = int(utc_time_str[0:2])
            minute = int(utc_time_str[2:4])
            second = int(utc_time_str[4:6])
            subsecond_str = utc_time_str.split('.')[1]
            nanosecond = int(subsecond_str.ljust(9, '0')[:9]) # Uzupełnij do 9 cyfr dla nanosekund

            # Stwórz obiekt datetime z dzisiejszą datą i czasem GPS
            gps_datetime = now_utc.replace(hour=hour, minute=minute, second=second, microsecond=nanosecond // 1000, tzinfo=timezone.utc)
            
            ros_time = GpsRtk().header.stamp # Uzyskaj pusty obiekt Time
            ros_time.sec = int(gps_datetime.timestamp())
            ros_time.nanosec = gps_datetime.microsecond * 1000 # nanosecond od początku sekundy
            return ros_time
        except ValueError as e:
            self.get_logger().warn(f"Błąd konwersji czasu GPS UTC '{utc_time_str}': {e}")
            return self.get_clock().now().to_msg()

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        bt_status = DiagnosticStatus()
        bt_status.name = 'Bluetooth Link (@traktor)'
        bt_status.hardware_id = self.host_mac

        if self.client_sock and self.client_info:
            bt_status.level = DiagnosticStatus.OK
            bt_status.message = 'Connected'
            client_mac = self.client_info[0]
            bt_status.values.append(KeyValue(key='Client Address', value=client_mac))
            bt_status.values.append(KeyValue(key='Connection Status', value='Connected'))

            try:
                cmd = ['hcitool', 'lq', client_mac]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0, check=True)
                output_line = result.stdout.strip()
                lq_value = output_line.split(':')[-1].strip()
                bt_status.values.append(KeyValue(key='Link Quality', value=lq_value))
                if int(lq_value) < 200:
                    bt_status.level = DiagnosticStatus.WARN
                    bt_status.message = 'Connected (Low Quality)'
            except Exception as e:
                bt_status.values.append(KeyValue(key='Link Quality', value='Error'))
                self.get_logger().debug(f"Nie udało się odczytać Link Quality: {e}")
        else:
            bt_status.level = DiagnosticStatus.WARN
            bt_status.message = 'Disconnected'
            bt_status.values.append(KeyValue(key='Connection Status', value='Disconnected'))

        diag_array.status.append(bt_status)
        self.diag_publisher_.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BluetoothReceiverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as e:
        if isinstance(e, ValueError): print(f"Błąd krytyczny: {e}")
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()