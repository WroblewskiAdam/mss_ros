# relative_position_computer/relative_computer_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
import numpy as np
import json
import psutil
import time
# Upewnij się, że nazwa pakietu z wiadomościami jest poprawna
from my_robot_interfaces.msg import GpsRtk, DistanceMetrics
from std_msgs.msg import String 

class RelativeComputerNode(Node):
    """
    Ten węzeł synchronizuje dane GPS z ciągnika i sieczkarni,
    oblicza odległość w linii prostej oraz odległości wzdłużną i poprzeczną
    względem toru jazdy sieczkarni, a następnie publikuje wyniki.
    Używa absolutnego kursu z GPS z dwiema antenami.
    """
    def __init__(self):
        super().__init__('relative_computer_node')

        self.declare_parameter('tractor_gps_topic', '/gps_rtk_data/tractor_filtered')
        self.declare_parameter('chopper_gps_topic', '/gps_rtk_data/chopper_filtered')
        self.declare_parameter('distance_metrics_topic', '/distance_metrics')
        self.declare_parameter('earth_radius_m', 6371000.0)

        tractor_topic = self.get_parameter('tractor_gps_topic').get_parameter_value().string_value
        chopper_topic = self.get_parameter('chopper_gps_topic').get_parameter_value().string_value
        metrics_topic = self.get_parameter('distance_metrics_topic').get_parameter_value().string_value
        self.R_EARTH = self.get_parameter('earth_radius_m').get_parameter_value().double_value

        self.origin_lat_rad = None
        self.origin_lon_rad = None
        self.is_origin_set = False

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.metrics_publisher = self.create_publisher(DistanceMetrics, metrics_topic, 10)

        self.tractor_sub = message_filters.Subscriber(self, GpsRtk, tractor_topic, qos_profile=sensor_qos_profile)
        self.chopper_sub = message_filters.Subscriber(self, GpsRtk, chopper_topic, qos_profile=sensor_qos_profile)

        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.tractor_sub, self.chopper_sub],
            queue_size=10,
            slop=0.1
        )
        self.time_synchronizer.registerCallback(self.synchronized_callback)

        # === NOWY PUBLISHER: Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/relative_computer_node', 10)
        # === NOWY TIMER: Health reporting co 5 sekund ===
        self.health_timer = self.create_timer(5.0, self.publish_health)

        self.get_logger().info("Węzeł obliczania pozycji względnej uruchomiony (tryb absolutnego kursu).")


    def latlon_to_enu(self, lat_deg, lon_deg):
        lat_rad = np.deg2rad(lat_deg)
        lon_rad = np.deg2rad(lon_deg)
        x = self.R_EARTH * (lon_rad - self.origin_lon_rad) * np.cos(self.origin_lat_rad)
        y = self.R_EARTH * (lat_rad - self.origin_lat_rad)
        return np.array([x, y])


    def synchronized_callback(self, tractor_msg, chopper_msg):
        if not self.is_origin_set:
            self.origin_lat_rad = np.deg2rad(chopper_msg.latitude_deg)
            self.origin_lon_rad = np.deg2rad(chopper_msg.longitude_deg)
            self.is_origin_set = True
            self.get_logger().info(f"Ustawiono punkt odniesienia ENU na: Lat={chopper_msg.latitude_deg}, Lon={chopper_msg.longitude_deg}")
            return

        heading_to_use_deg = chopper_msg.heading_deg

        tractor_pos_enu = self.latlon_to_enu(tractor_msg.latitude_deg, tractor_msg.longitude_deg)
        chopper_pos_enu = self.latlon_to_enu(chopper_msg.latitude_deg, chopper_msg.longitude_deg)

        vector_chopper_to_tractor = tractor_pos_enu - chopper_pos_enu
        dist_straight = np.linalg.norm(vector_chopper_to_tractor)

        heading_rad = np.deg2rad(heading_to_use_deg)
        chopper_heading_vector = np.array([np.sin(heading_rad), np.cos(heading_rad)])
        
        dist_longitudinal = np.dot(vector_chopper_to_tractor, chopper_heading_vector)
        dist_lateral = np.cross(chopper_heading_vector, vector_chopper_to_tractor)

        metrics_msg = DistanceMetrics()
        
        # === ZMIANA: Dodajemy znacznik czasu ===
        metrics_msg.header.stamp = self.get_clock().now().to_msg()
        # ======================================

        # Rzutowanie na standardowy typ float
        metrics_msg.distance_straight = float(dist_straight)
        metrics_msg.distance_longitudinal = float(dist_longitudinal)
        metrics_msg.distance_lateral = float(dist_lateral)
        
        self.metrics_publisher.publish(metrics_msg)

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status publisherów
            publisher_status = "OK" if hasattr(self, 'metrics_publisher') else "ERROR"
            
            # Sprawdź status subskrypcji
            subscription_status = "OK"
            if not hasattr(self, 'tractor_sub') or not hasattr(self, 'chopper_sub'):
                subscription_status = "ERROR"
            
            # Sprawdź status synchronizatora
            synchronizer_status = "OK" if hasattr(self, 'time_synchronizer') else "ERROR"
            
            # Sprawdź status punktu odniesienia
            origin_status = "SET" if self.is_origin_set else "NOT_SET"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if publisher_status == "ERROR":
                errors.append("Publisher nieaktywny")
            if subscription_status == "ERROR":
                errors.append("Subskrypcje nieaktywne")
            if synchronizer_status == "ERROR":
                errors.append("Synchronizator nieaktywny")
            if origin_status == "NOT_SET":
                warnings.append("Punkt odniesienia nie ustawiony")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'publisher_status': publisher_status,
                'subscription_status': subscription_status,
                'synchronizer_status': synchronizer_status,
                'origin_status': origin_status,
                'earth_radius_m': self.R_EARTH,
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


def main(args=None):
    rclpy.init(args=args)
    node = RelativeComputerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()