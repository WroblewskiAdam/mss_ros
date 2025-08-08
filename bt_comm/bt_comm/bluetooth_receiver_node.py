# FINALNA WERSJA ODBIORNIKA (ciągnik) - z poprawką konwersji czasu i błędu "bad address"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from my_robot_interfaces.msg import GpsRtk
import socket
import json
import threading
import time
from datetime import datetime, timezone # <--- WAŻNY IMPORT

class BluetoothReceiverNode(Node):
    def __init__(self):
        super().__init__('bluetooth_receiver_node')
        
        self.declare_parameter('bt_port', 4)
        self.declare_parameter('publish_topic', '/gps_rtk_data/chopper')

        self.port = self.get_parameter('bt_port').get_parameter_value().integer_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 
        )
        self.publisher_ = self.create_publisher(GpsRtk, self.publish_topic, qos_profile)

        self.stop_event = threading.Event()
        self.server_thread = threading.Thread(target=self.server_loop, daemon=True)
        self.server_thread.start()
        self.get_logger().info("Wątek serwera Bluetooth uruchomiony.")

    def _convert_gps_utc_to_ros_time(self, utc_time_str: str):
        """Konwertuje czas UTC z GNGGA (hhmmss.ss) na ROS Time."""
        if not utc_time_str or '.' not in utc_time_str:
            return self.get_clock().now().to_msg() # Zwróć aktualny czas ROS jako fallback

        try:
            now_utc = datetime.now(timezone.utc)
            
            hour = int(utc_time_str[0:2])
            minute = int(utc_time_str[2:4])
            second = int(utc_time_str[4:6])
            
            # Poprawna obsługa nanosekund
            subsecond_str = utc_time_str.split('.')[1]
            nanosecond = int(subsecond_str.ljust(9, '0')[:9])

            gps_datetime = now_utc.replace(
                hour=hour, minute=minute, second=second, microsecond=0, tzinfo=timezone.utc
            )
            
            # Uzyskaj pusty obiekt Time z wiadomości GpsRtk
            ros_time = GpsRtk().gps_time 
            ros_time.sec = int(gps_datetime.timestamp())
            ros_time.nanosec = nanosecond
            return ros_time
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Błąd konwersji czasu GPS UTC '{utc_time_str}': {e}")
            return self.get_clock().now().to_msg()

    def server_loop(self):
        """Główna pętla serwera, działająca w osobnym wątku."""
        server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((socket.BDADDR_ANY, self.port))
        server_sock.listen(1)
        self.get_logger().info(f"Serwer Bluetooth nasłuchuje na porcie {self.port}...")

        while not self.stop_event.is_set():
            try:
                client_sock, client_info = server_sock.accept()
                self.get_logger().info(f"Zaakceptowano połączenie od: {client_info}")
                
                buffer = ""
                while not self.stop_event.is_set():
                    data = client_sock.recv(1024)
                    if not data:
                        self.get_logger().warn("Klient zamknął połączenie.")
                        break

                    buffer += data.decode('utf-8', errors='ignore')
                    
                    while '\n' in buffer:
                        message, buffer = buffer.split('\n', 1)
                        if not message: continue

                        try:
                            gps_data = json.loads(message)
                            msg = GpsRtk()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = "chopper_gps_link"
                            
                            msg.latitude_deg = float(gps_data.get('lat', 0.0))
                            msg.longitude_deg = float(gps_data.get('lon', 0.0))
                            msg.speed_mps = float(gps_data.get('speed', 0.0))
                            msg.heading_deg = float(gps_data.get('heading', 0.0))
                            msg.rtk_status = int(gps_data.get('rtk_status', 0))
                            msg.altitude_m = float(gps_data.get('altitude', 0.0))
                            
                            # --- KRYTYCZNA POPRAWKA CZASU ---
                            msg.gps_time = self._convert_gps_utc_to_ros_time(gps_data.get('gps_time', ""))
                            # -------------------------------

                            self.publisher_.publish(msg)
                            
                        except json.JSONDecodeError:
                            self.get_logger().warn(f"Pominięto niepoprawny fragment JSON: '{message}'")
                        except Exception as e:
                            self.get_logger().error(f"Błąd przetwarzania wiadomości: {e}")

            except socket.error as e:
                self.get_logger().error(f"Błąd gniazda: {e}. Oczekuję 5s.")
                time.sleep(5)
            except Exception as e:
                self.get_logger().fatal(f"Krytyczny błąd pętli serwera: {e}")
                break
        
        server_sock.close()

    def destroy_node(self):
        self.get_logger().info("Zatrzymywanie wątku serwera Bluetooth...")
        self.stop_event.set()
        if self.server_thread.is_alive():
            self.server_thread.join(timeout=1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()