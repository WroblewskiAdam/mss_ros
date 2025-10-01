import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 
from scipy.signal import butter, lfilter_zi, lfilter, medfilt
import numpy as np
import json
import psutil
import time
from collections import deque

class ChopperFilterNode(Node):
    def __init__(self):
        super().__init__('chopper_filter_node')

        # --- Parametry filtru prędkości ---
        self.declare_parameter('speed_filter_cutoff_hz', 0.8)
        self.declare_parameter('filter_order', 2)
        self.declare_parameter('sampling_frequency_hz', 20.0)
        
        # --- Parametry filtru pozycji ---
        self.declare_parameter('position_filter_cutoff_hz', 0.5)
        
        # --- Parametry filtru headingu ---
        self.declare_parameter('heading_filter_cutoff_hz', 0.3)
        
        # --- Parametry filtra medianowego ---
        self.declare_parameter('median_filter_window', 5)
        self.declare_parameter('median_filter_on_off', True)  # Parametr włączania/wyłączania filtra medianowego

        # Parametry filtru prędkości
        speed_fpass = self.get_parameter('speed_filter_cutoff_hz').get_parameter_value().double_value
        order = self.get_parameter('filter_order').get_parameter_value().integer_value
        fs = self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value
        
        # Parametry filtru pozycji
        position_fpass = self.get_parameter('position_filter_cutoff_hz').get_parameter_value().double_value
        
        # Parametry filtru headingu
        heading_fpass = self.get_parameter('heading_filter_cutoff_hz').get_parameter_value().double_value
        
        # Parametry filtra medianowego
        self.median_window = self.get_parameter('median_filter_window').get_parameter_value().integer_value
        self.median_filter_enabled = self.get_parameter('median_filter_on_off').get_parameter_value().bool_value

        # QoS dla danych GPS
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subskrypcja surowych danych GPS sieczkarni
        self.subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/chopper',
            self.listener_callback,
            qos_profile=sensor_qos_profile)

        # Publikacja przefiltrowanych danych GPS sieczkarni
        self.publisher_ = self.create_publisher(
            GpsRtk, 
            '/gps_rtk_data/chopper_filtered', 
            qos_profile=sensor_qos_profile
        )

        # === Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/chopper_filter_node', 10)
        self.health_timer = self.create_timer(5.0, self.publish_health)

        # Inicjalizacja filtru Butterworth dla prędkości
        self.speed_b, self.speed_a = butter(order, speed_fpass, btype='low', analog=False, fs=fs)
        self.speed_zi = lfilter_zi(self.speed_b, self.speed_a)
        
        # Inicjalizacja filtrów Butterworth dla pozycji (lat/lon)
        self.position_b, self.position_a = butter(order, position_fpass, btype='low', analog=False, fs=fs)
        self.lat_zi = lfilter_zi(self.position_b, self.position_a)
        self.lon_zi = lfilter_zi(self.position_b, self.position_a)
        
        # Inicjalizacja filtru Butterworth dla headingu
        self.heading_b, self.heading_a = butter(order, heading_fpass, btype='low', analog=False, fs=fs)
        self.heading_zi = lfilter_zi(self.heading_b, self.heading_a)
        
        # Inicjalizacja buforów dla filtra medianowego
        self.speed_buffer = deque(maxlen=self.median_window)
        self.lat_buffer = deque(maxlen=self.median_window)
        self.lon_buffer = deque(maxlen=self.median_window)
        self.heading_buffer = deque(maxlen=self.median_window)

        # Statystyki
        self.filtered_count = 0
        self.last_filter_time = time.time()

        self.get_logger().info(f"Filtr sieczkarni uruchomiony.")
        self.get_logger().info(f"Parametry filtru prędkości: cutoff={speed_fpass}Hz, order={order}, fs={fs}Hz")
        self.get_logger().info(f"Parametry filtru pozycji: cutoff={position_fpass}Hz, order={order}, fs={fs}Hz")
        self.get_logger().info(f"Parametry filtru headingu: cutoff={heading_fpass}Hz, order={order}, fs={fs}Hz")
        self.get_logger().info(f"Parametry filtra medianowego: window={self.median_window}, enabled={self.median_filter_enabled}")

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status filtrów
            filter_status = "OK"
            if not hasattr(self, 'speed_b') or not hasattr(self, 'speed_a') or not hasattr(self, 'position_b') or not hasattr(self, 'position_a') or not hasattr(self, 'heading_b') or not hasattr(self, 'heading_a'):
                filter_status = "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if filter_status == "ERROR":
                errors.append("Filtry nie zainicjalizowane")
            
            # Sprawdź czy dane są odbierane
            time_since_last_filter = time.time() - self.last_filter_time
            if time_since_last_filter > 10.0:  # Brak danych przez 10s
                warnings.append(f"Brak danych GPS sieczkarni przez {time_since_last_filter:.1f}s")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'filter_status': filter_status,
                'speed_filter_cutoff_hz': self.get_parameter('speed_filter_cutoff_hz').get_parameter_value().double_value,
                'position_filter_cutoff_hz': self.get_parameter('position_filter_cutoff_hz').get_parameter_value().double_value,
                'heading_filter_cutoff_hz': self.get_parameter('heading_filter_cutoff_hz').get_parameter_value().double_value,
                'filter_order': self.get_parameter('filter_order').get_parameter_value().integer_value,
                'sampling_frequency_hz': self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value,
                'median_filter_window': self.get_parameter('median_filter_window').get_parameter_value().integer_value,
                'median_filter_enabled': self.get_parameter('median_filter_on_off').get_parameter_value().bool_value,
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

    def normalize_heading(self, heading_deg):
        """Normalizuje heading do zakresu 0-360 stopni."""
        while heading_deg < 0:
            heading_deg += 360
        while heading_deg >= 360:
            heading_deg -= 360
        return heading_deg

    def listener_callback(self, msg):
        """Callback dla surowych danych GPS sieczkarni."""
        self.get_logger().debug(f'>>> Otrzymano dane GPS sieczkarni! Prędkość surowa: {msg.speed_mps:.2f} m/s, Lat: {msg.latitude_deg:.8f}, Lon: {msg.longitude_deg:.8f}, Heading: {msg.heading_deg:.2f}°')
        
        # Sprawdź czy filtr medianowy jest włączony
        if self.median_filter_enabled:
            # Dodaj surowe dane do buforów filtra medianowego
            self.speed_buffer.append(msg.speed_mps)
            self.lat_buffer.append(msg.latitude_deg)
            self.lon_buffer.append(msg.longitude_deg)
            self.heading_buffer.append(self.normalize_heading(msg.heading_deg))
            
            # Zastosuj filtr medianowy
            median_speed = np.median(list(self.speed_buffer))
            median_lat = np.median(list(self.lat_buffer))
            median_lon = np.median(list(self.lon_buffer))
            median_heading = np.median(list(self.heading_buffer))
            
            # Filtruj prędkość (najpierw medianowy, potem Butterworth)
            filtered_speed, self.speed_zi = lfilter(self.speed_b, self.speed_a, [median_speed], zi=self.speed_zi)
            
            # Filtruj pozycję (najpierw medianowy, potem Butterworth)
            filtered_lat, self.lat_zi = lfilter(self.position_b, self.position_a, [median_lat], zi=self.lat_zi)
            filtered_lon, self.lon_zi = lfilter(self.position_b, self.position_a, [median_lon], zi=self.lon_zi)
            
            # Filtruj heading (najpierw medianowy, potem Butterworth)
            filtered_heading, self.heading_zi = lfilter(self.heading_b, self.heading_a, [median_heading], zi=self.heading_zi)
        else:
            # Filtr medianowy wyłączony - używaj surowych danych
            # Filtruj prędkość (tylko Butterworth)
            filtered_speed, self.speed_zi = lfilter(self.speed_b, self.speed_a, [msg.speed_mps], zi=self.speed_zi)
            
            # Filtruj pozycję (tylko Butterworth)
            filtered_lat, self.lat_zi = lfilter(self.position_b, self.position_a, [msg.latitude_deg], zi=self.lat_zi)
            filtered_lon, self.lon_zi = lfilter(self.position_b, self.position_a, [msg.longitude_deg], zi=self.lon_zi)
            
            # Filtruj heading (tylko Butterworth)
            filtered_heading, self.heading_zi = lfilter(self.heading_b, self.heading_a, [self.normalize_heading(msg.heading_deg)], zi=self.heading_zi)

        # Stwórz nową wiadomość z przefiltrowanymi danymi
        filtered_msg = GpsRtk()
        filtered_msg.header = msg.header
        filtered_msg.gps_time = msg.gps_time
        filtered_msg.rtk_status = msg.rtk_status
        filtered_msg.latitude_deg = filtered_lat[0]  # Filtrowana pozycja
        filtered_msg.longitude_deg = filtered_lon[0]  # Filtrowana pozycja
        filtered_msg.altitude_m = msg.altitude_m
        filtered_msg.heading_deg = self.normalize_heading(filtered_heading[0])  # Filtrowany heading
        filtered_msg.speed_mps = filtered_speed[0]  # Filtrowana prędkość

        self.publisher_.publish(filtered_msg)
        
        # Aktualizuj statystyki
        self.filtered_count += 1
        self.last_filter_time = time.time()
        
        # Logowanie z informacją o stanie filtra medianowego
        if self.median_filter_enabled:
            self.get_logger().debug(f'Przefiltrowane dane sieczkarni: Speed={filtered_speed[0]:.2f} m/s (median: {median_speed:.2f}), Lat={filtered_lat[0]:.8f}, Lon={filtered_lon[0]:.8f}, Heading={self.normalize_heading(filtered_heading[0]):.2f}° (median: {median_heading:.2f}°)')
        else:
            self.get_logger().debug(f'Przefiltrowane dane sieczkarni: Speed={filtered_speed[0]:.2f} m/s (raw: {msg.speed_mps:.2f}, median OFF), Lat={filtered_lat[0]:.8f}, Lon={filtered_lon[0]:.8f}, Heading={self.normalize_heading(filtered_heading[0]):.2f}° (raw: {msg.heading_deg:.2f}°, median OFF)')

def main(args=None):
    rclpy.init(args=args)
    node = ChopperFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
