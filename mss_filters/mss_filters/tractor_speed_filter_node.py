import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 
from scipy.signal import butter, lfilter_zi, lfilter
import numpy as np
import json
import psutil
import time

class TractorSpeedFilterNode(Node):
    def __init__(self):
        super().__init__('tractor_speed_filter_node')

        # --- Parametry filtru ---
        self.declare_parameter('filter_cutoff_hz', 0.8)
        self.declare_parameter('filter_order', 2)
        self.declare_parameter('sampling_frequency_hz', 20.0)

        fpass = self.get_parameter('filter_cutoff_hz').get_parameter_value().double_value
        order = self.get_parameter('filter_order').get_parameter_value().integer_value
        fs = self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value

        # QoS dla danych GPS
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subskrypcja surowych danych GPS ciągnika
        self.subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/tractor',
            self.listener_callback,
            qos_profile=sensor_qos_profile)

        # Publikacja przefiltrowanych danych GPS ciągnika
        self.publisher_ = self.create_publisher(
            GpsRtk, 
            '/gps_rtk_data/tractor_filtered', 
            qos_profile=sensor_qos_profile
        )

        # === Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/tractor_speed_filter_node', 10)
        self.health_timer = self.create_timer(5.0, self.publish_health)

        # Inicjalizacja filtru Butterworth
        self.b, self.a = butter(order, fpass, btype='low', analog=False, fs=fs)
        self.zi = lfilter_zi(self.b, self.a)

        # Statystyki
        self.filtered_count = 0
        self.last_filter_time = time.time()

        self.get_logger().info(f"Filtr prędkości ciągnika uruchomiony.")
        self.get_logger().info(f"Parametry filtru: cutoff={fpass}Hz, order={order}, fs={fs}Hz")

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status filtru
            filter_status = "OK"
            if not hasattr(self, 'b') or not hasattr(self, 'a'):
                filter_status = "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if filter_status == "ERROR":
                errors.append("Filtr nie zainicjalizowany")
            
            # Sprawdź czy dane są odbierane
            time_since_last_filter = time.time() - self.last_filter_time
            if time_since_last_filter > 10.0:  # Brak danych przez 10s
                warnings.append(f"Brak danych GPS ciągnika przez {time_since_last_filter:.1f}s")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'filter_status': filter_status,
                'filter_cutoff_hz': self.get_parameter('filter_cutoff_hz').get_parameter_value().double_value,
                'filter_order': self.get_parameter('filter_order').get_parameter_value().integer_value,
                'sampling_frequency_hz': self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value,
                'filtered_messages_count': self.filtered_count,
                'time_since_last_data': time_since_last_filter,
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

    def listener_callback(self, msg):
        """Callback dla surowych danych GPS ciągnika."""
        self.get_logger().debug(f'>>> Otrzymano dane GPS ciągnika! Prędkość surowa: {msg.speed_mps:.2f} m/s')
        
        # Filtruj tylko prędkość, pozostaw inne dane bez zmian
        raw_speed = msg.speed_mps
        filtered_speed, self.zi = lfilter(self.b, self.a, [raw_speed], zi=self.zi)

        # Stwórz nową wiadomość z przefiltrowaną prędkością
        filtered_msg = GpsRtk()
        filtered_msg.header = msg.header
        filtered_msg.gps_time = msg.gps_time
        filtered_msg.rtk_status = msg.rtk_status
        filtered_msg.latitude_deg = msg.latitude_deg
        filtered_msg.longitude_deg = msg.longitude_deg
        filtered_msg.altitude_m = msg.altitude_m
        filtered_msg.heading_deg = msg.heading_deg
        filtered_msg.speed_mps = filtered_speed[0]  # Tylko prędkość jest filtrowana

        self.publisher_.publish(filtered_msg)
        
        # Aktualizuj statystyki
        self.filtered_count += 1
        self.last_filter_time = time.time()
        
        self.get_logger().debug(f'Przefiltrowana prędkość ciągnika: {filtered_speed[0]:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = TractorSpeedFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()