# relative_position_computer/relative_computer_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
import numpy as np
from my_robot_interfaces.msg import GpsRtk, DistanceMetrics

class RelativeComputerNode(Node):
    """
    Ten węzeł synchronizuje dane GPS z ciągnika i sieczkarni,
    oblicza odległość w linii prostej oraz odległości wzdłużną i poprzeczną
    względem toru jazdy sieczkarni, a następnie publikuje wyniki.
    Implementuje logikę stabilizacji kierunku (heading) przy niskich prędkościach.
    """
    def __init__(self):
        super().__init__('relative_computer_node')

        self.declare_parameter('tractor_gps_topic', '/gps_rtk_data')
        self.declare_parameter('chopper_gps_topic', '/gps_rtk_data/chopper')
        self.declare_parameter('distance_metrics_topic', '/distance_metrics')
        self.declare_parameter('earth_radius_m', 6371000.0)
        self.declare_parameter('heading_speed_threshold_ms', 0.1)

        tractor_topic = self.get_parameter('tractor_gps_topic').get_parameter_value().string_value
        chopper_topic = self.get_parameter('chopper_gps_topic').get_parameter_value().string_value
        metrics_topic = self.get_parameter('distance_metrics_topic').get_parameter_value().string_value
        self.R_EARTH = self.get_parameter('earth_radius_m').get_parameter_value().double_value
        self.heading_speed_threshold = self.get_parameter('heading_speed_threshold_ms').get_parameter_value().double_value

        self.origin_lat_rad = None
        self.origin_lon_rad = None
        self.is_origin_set = False
        self.last_valid_chopper_heading_deg = None

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
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

        self.get_logger().info("Węzeł obliczania pozycji względnej uruchomiony (z logiką stabilizacji kierunku).")
        self.get_logger().info(f"Próg prędkości dla aktualizacji kierunku: {self.heading_speed_threshold} m/s")


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

        chopper_speed_ms = chopper_msg.speed_mps
        
        heading_to_use_deg = self.last_valid_chopper_heading_deg

        if chopper_speed_ms > self.heading_speed_threshold:
            self.last_valid_chopper_heading_deg = chopper_msg.heading_deg
            heading_to_use_deg = self.last_valid_chopper_heading_deg
        else:
            if self.last_valid_chopper_heading_deg is None:
                self.get_logger().warn(
                    "Prędkość sieczkarni jest poniżej progu, a brak zapisanego wiarygodnego kierunku. Pomijam pomiar.",
                    throttle_duration_sec=10
                )
                return
            
            self.get_logger().debug(
                f"Prędkość {chopper_speed_ms:.2f} m/s jest poniżej progu. Używam zapisanego kierunku: {heading_to_use_deg:.2f} deg."
            )

        tractor_pos_enu = self.latlon_to_enu(tractor_msg.latitude_deg, tractor_msg.longitude_deg)
        chopper_pos_enu = self.latlon_to_enu(chopper_msg.latitude_deg, chopper_msg.longitude_deg)

        vector_chopper_to_tractor = tractor_pos_enu - chopper_pos_enu
        dist_straight = np.linalg.norm(vector_chopper_to_tractor)

        heading_rad = np.deg2rad(heading_to_use_deg)
        chopper_heading_vector = np.array([np.sin(heading_rad), np.cos(heading_rad)])
        
        dist_longitudinal = np.dot(vector_chopper_to_tractor, chopper_heading_vector)
        dist_lateral = np.cross(chopper_heading_vector, vector_chopper_to_tractor)

        metrics_msg = DistanceMetrics()
        
        # --- TUTAJ NASTĄPIŁA ZMIANA ---
        # Rzutujemy jawnie każdy wynik z NumPy na standardowy typ float
        metrics_msg.distance_straight = float(dist_straight)
        metrics_msg.distance_longitudinal = float(dist_longitudinal)
        metrics_msg.distance_lateral = float(dist_lateral)
        
        self.metrics_publisher.publish(metrics_msg)


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