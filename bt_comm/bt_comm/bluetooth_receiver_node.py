# Wersja finalna: Zoptymalizowana, niezawodna, z czystym zamykaniem
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import String
import socket
import json
import threading
import time
from datetime import datetime, timezone
import psutil

class BluetoothReceiverNode(Node):
    def __init__(self):
        super().__init__('bluetooth_receiver_node')
        
        self.declare_parameter('bt_port', 1)  # Zmiana z portu 1 na 2
        self.declare_parameter('publish_topic', '/gps_rtk_data/chopper')
        # === NOWE PARAMETRY ===
        self.declare_parameter('connection_timeout', 0.5)  # 500ms zamiast 1s
        self.declare_parameter('health_report_interval', 5)  # 10s zamiast 5s
        self.declare_parameter('reconnect_delay', 2.0)

        self.port = self.get_parameter('bt_port').get_parameter_value().integer_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.health_report_interval = self.get_parameter('health_report_interval').get_parameter_value().double_value
        self.reconnect_delay = self.get_parameter('reconnect_delay').get_parameter_value().double_value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10 
        )
        self.publisher_ = self.create_publisher(GpsRtk, self.publish_topic, qos_profile)

        # === OPTYMALIZACJA: Health reporting co 10 sekund zamiast 5 ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/bt_receiver_node', qos_profile)
        self.health_timer = self.create_timer(self.health_report_interval, self.publish_health)

        # === NOWY TIMER: Logowanie statusu połączenia co 2 sekundy ===
        self.connection_log_timer = self.create_timer(2.0, self.log_connection_status)

        # === NOWE ZMIENNE STANU ===
        self.connection_status = "DISCONNECTED"
        self.last_connection_time = 0.0
        self.data_received_count = 0
        self.last_data_time = 0.0
        self.last_connection_log_time = 0.0  # Dodane do kontroli logowania

        self.stop_event = threading.Event()
        self.server_thread = threading.Thread(target=self.server_loop, daemon=True)
        self.server_thread.start()
        self.get_logger().info("Wątek serwera Bluetooth uruchomiony.")

    def log_connection_status(self):
        """Loguje status połączenia co 2 sekundy, żeby nie zaśmiecać terminala."""
        current_time = time.time()
        
        # Loguj tylko jeśli minęło 2 sekundy od ostatniego logu
        if current_time - self.last_connection_log_time >= 2.0:
            if self.connection_status == "WAITING":
                self.get_logger().info("Oczekiwanie na połączenie od ESP32...")
            elif self.connection_status == "DISCONNECTED":
                self.get_logger().info("Status: Brak połączenia Bluetooth")
            elif self.connection_status == "CONNECTED":
                # Loguj połączenie tylko raz przy nawiązaniu
                pass
            
            self.last_connection_log_time = current_time

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status wątku serwera
            server_thread_status = "OK" if self.server_thread.is_alive() else "ERROR"
            
            # Sprawdź status połączenia Bluetooth
            bt_status = self.connection_status
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if server_thread_status == "ERROR":
                errors.append("Wątek serwera nieaktywny")
            if bt_status == "DISCONNECTED":
                warnings.append("Brak połączenia Bluetooth")
            if bt_status == "WAITING":
                # To jest normalny stan dla serwera - oczekuje na połączenie
                pass
            
            # Sprawdź świeżość danych
            current_time = time.time()
            if self.last_data_time > 0:
                time_since_data = current_time - self.last_data_time
                if time_since_data > 30.0:  # 30 sekund bez danych
                    warnings.append(f"Brak danych przez {time_since_data:.1f}s")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': current_time,
                'server_thread_status': server_thread_status,
                'bt_status': bt_status,
                'port': self.port,
                'data_received_count': self.data_received_count,
                'last_data_age': current_time - self.last_data_time if self.last_data_time > 0 else 0,
                'errors': errors,
                'warnings': warnings,
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent
            }
            
            # Opublikuj health status
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas publikowania health status: {e}")

    def _convert_gps_utc_to_ros_time(self, utc_time_str: str):
        if not utc_time_str or '.' not in utc_time_str:
            return self.get_clock().now().to_msg() 

        try:
            now_utc = datetime.now(timezone.utc)
            hour = int(utc_time_str[0:2])
            minute = int(utc_time_str[2:4])
            second = int(utc_time_str[4:6])
            subsecond_str = utc_time_str.split('.')[1]
            nanosecond = int(subsecond_str.ljust(9, '0')[:9])
            gps_datetime = now_utc.replace(
                hour=hour, minute=minute, second=second, microsecond=0, tzinfo=timezone.utc
            )
            ros_time = GpsRtk().gps_time 
            ros_time.sec = int(gps_datetime.timestamp())
            ros_time.nanosec = nanosecond
            return ros_time
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Błąd konwersji czasu GPS UTC '{utc_time_str}': {e}")
            return self.get_clock().now().to_msg()

    def server_loop(self):
        """Zoptymalizowana pętla serwera Bluetooth z lepszym zarządzaniem połączeniami."""
        with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM) as server_sock:
            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_sock.bind((socket.BDADDR_ANY, self.port))
            server_sock.listen(1)
            
            # === OPTYMALIZACJA: Non-blocking socket ===
            server_sock.settimeout(0.1)  # 100ms timeout zamiast blokującego accept()
            
            self.get_logger().info(f"Serwer Bluetooth nasłuchuje na porcie {self.port}...")

            while not self.stop_event.is_set():
                client_sock = None
                try:
                    # === OPTYMALIZACJA: Non-blocking accept z timeout ===
                    try:
                        client_sock, client_info = server_sock.accept()
                        self.get_logger().info(f"Zaakceptowano połączenie od: {client_info}")
                        self.connection_status = "CONNECTED"
                        self.last_connection_time = time.time()
                        self.last_connection_log_time = time.time()  # Reset timer'a logowania
                    except socket.timeout:
                        # Timeout - sprawdź czy system ma się zatrzymać
                        continue
                    except socket.error as e:
                        if not self.stop_event.is_set():
                            self.get_logger().warn(f"Błąd accept: {e}")
                        continue
                    
                    # === OPTYMALIZACJA: Krótszy timeout dla klienta ===
                    client_sock.settimeout(self.connection_timeout)

                    buffer = ""
                    while not self.stop_event.is_set():
                        try:
                            data = client_sock.recv(1024)
                            
                            if not data:
                                self.get_logger().warn("Klient zamknął połączenie (recv zwrócił 0 bajtów).")
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
                                    msg.gps_time = self._convert_gps_utc_to_ros_time(gps_data.get('gps_time', ""))

                                    self.publisher_.publish(msg)
                                    
                                    # === AKTUALIZACJA STATYSTYK ===
                                    self.data_received_count += 1
                                    self.last_data_time = time.time()
                                    
                                except json.JSONDecodeError:
                                    self.get_logger().warn(f"Pominięto niepoprawny fragment JSON: '{message}'")
                                except Exception as e:
                                    self.get_logger().error(f"Błąd przetwarzania wiadomości: {e}")

                        except socket.timeout:
                            # === OPTYMALIZACJA: Krótszy timeout, szybsza reakcja ===
                            self.get_logger().debug(f"Timeout - nie otrzymano danych od {client_info}")
                            break 

                except socket.error as e:
                    # Ten błąd wystąpi tylko, jeśli problem ma serwer (nie klient)
                    # lub jeśli stop_event przerwie .accept()
                    if not self.stop_event.is_set():
                        self.get_logger().error(f"Błąd gniazda w pętli serwera: {e}. Czekam {self.reconnect_delay}s.")
                        time.sleep(self.reconnect_delay)
                except Exception as e:
                    self.get_logger().fatal(f"Krytyczny, nieoczekiwany błąd pętli serwera: {e}")
                    break
                finally:
                    if client_sock:
                        client_sock.close()
                        self.connection_status = "DISCONNECTED"
                        self.get_logger().info("Połączenie Bluetooth zostało przerwane")
                    
                    # === OPTYMALIZACJA: Serwer zawsze oczekuje na nowe połączenia ===
                    if not self.stop_event.is_set():
                        self.connection_status = "WAITING"
                        # Usunięto bezpośrednie logowanie - teraz kontrolowane przez timer

    def destroy_node(self):
        self.get_logger().info("Inicjowanie zamykania węzła...")

        self.stop_event.set()

        try:
            with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM) as dummy_socket:
                dummy_socket.settimeout(0.1)
                dummy_socket.connect((socket.BDADDR_LOCAL, self.port))
        except Exception:
            pass
        
        if hasattr(self, 'server_thread') and self.server_thread.is_alive():
            self.get_logger().info("Oczekiwanie na zamknięcie wątku serwera Bluetooth...")
            self.server_thread.join(timeout=1.5) 
            if self.server_thread.is_alive():
                self.get_logger().warn("Wątek serwera BT nie zamknął się w wyznaczonym czasie.")

        super().destroy_node()
        print(f"[{self.get_name()}] Węzeł zamknięty pomyślnie.")

def main(args=None):
    rclpy.init(args=args)
    
    node = BluetoothReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Przechwycono KeyboardInterrupt. Rozpoczynanie zamykania...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
