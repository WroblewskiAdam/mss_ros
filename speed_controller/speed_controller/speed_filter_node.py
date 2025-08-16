import rclpy
from rclpy.node import Node
# from gps_rtk_msgs.msg import GpsRtk
from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import Float64, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 
from scipy.signal import butter, lfilter_zi, lfilter
import numpy as np
import json
import psutil
import time

class SpeedFilterNode(Node):
    def __init__(self):
        super().__init__('speed_filter_node')

        # --- Parametry filtru ---
        self.declare_parameter('filter_cutoff_hz', 0.8)
        self.declare_parameter('filter_order', 2)
        self.declare_parameter('sampling_frequency_hz', 20.0)

        fpass = self.get_parameter('filter_cutoff_hz').get_parameter_value().double_value
        order = self.get_parameter('filter_order').get_parameter_value().integer_value
        fs = self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data',
            self.listener_callback,
            qos_profile=sensor_qos_profile)

        self.publisher_ = self.create_publisher(GpsRtk, '/gps_rtk_data_filtered', qos_profile=sensor_qos_profile)

        # === NOWY PUBLISHER: Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/speed_filter_node', 10)
        # === NOWY TIMER: Health reporting co 5 sekund ===
        self.health_timer = self.create_timer(5.0, self.publish_health)

        self.b, self.a = butter(order, fpass, btype='low', analog=False, fs=fs)
        self.zi = lfilter_zi(self.b, self.a)

        self.get_logger().info(f"Filtr prędkości uruchomiony.")

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
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'filter_status': filter_status,
                'filter_cutoff_hz': self.get_parameter('filter_cutoff_hz').get_parameter_value().double_value,
                'filter_order': self.get_parameter('filter_order').get_parameter_value().integer_value,
                'sampling_frequency_hz': self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value,
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
        self.get_logger().debug(f'>>> Otrzymano dane w callbacku! Prędkość surowa: {msg.speed_mps:.2f}')
        
        raw_speed = msg.speed_mps
        filtered_speed, self.zi = lfilter(self.b, self.a, [raw_speed], zi=self.zi)

        filtered_msg = msg 
        filtered_msg.speed_mps = filtered_speed[0]

        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()