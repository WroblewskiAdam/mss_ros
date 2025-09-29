#!/usr/bin/env python3
"""
Chopper Distance Simulator Node - Symuluje sieczkarnię na podstawie całkowania różnicy prędkości
"""

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GpsRtk, DistanceMetrics
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import random
import time

class ChopperDistanceSimulatorNode(Node):
    def __init__(self):
        super().__init__('chopper_distance_simulator_node')
        
        # --- PARAMETRY SYMULACJI ---
        self.chopper_speed_kmh = 8.0           # Stała prędkość sieczkarni w km/h
        self.chopper_speed_mps = self.chopper_speed_kmh / 3.6  # Przeliczenie na m/s
        self.initial_distance_m = 0.0          # Początkowa odległość wzdłużna (0 = ta sama pozycja)
        self.min_distance_m = -50.0            # Minimalna odległość (ujemna = sieczkarnia przed ciągnikiem)
        self.max_distance_m = 80.0             # Maksymalna odległość
        
        # --- PARAMETRY ODLĘGŁOŚCI POPRZECZNEJ ---
        self.lateral_distance_m = 6.0          # Stała odległość poprzeczna
        self.lateral_noise_std = 0.01          # Szum odległości poprzecznej (1 cm)
        
        # --- PARAMETRY SZUMU GPS RTK ---
        self.gps_speed_noise_std = 0.03        # Szum prędkości GPS RTK Fix (m/s)
        self.gps_position_noise_std = 0.5      # Szum pozycji GPS RTK Fix (m)
        self.rtk_status = 4                    # Status RTK (4 = Fixed)
        
        # --- STAN WĘZŁA ---
        self.current_distance_m = self.initial_distance_m
        self.last_tractor_position = None
        self.last_tractor_speed = 0.0
        self.start_time = time.time()
        
        # --- QoS ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- SUBSKRYPCJE ---
        self.tractor_gps_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/tractor_filtered',
            self.tractor_gps_callback,
            qos_profile
        )
        
        # --- PUBLISHERS ---
        self.chopper_gps_publisher = self.create_publisher(
            GpsRtk,
            '/gps_rtk_data/chopper',
            qos_profile
        )
        
        self.distance_metrics_publisher = self.create_publisher(
            DistanceMetrics,
            '/distance_metrics',
            qos_profile
        )
        
        # --- TIMER ---
        self.publish_timer = self.create_timer(0.05, self.publish_chopper_data)  # 20 Hz
        
        # --- LOGOWANIE ---
        self.get_logger().info('=== CHOPPER DISTANCE SIMULATOR NODE URUCHOMIONY ===')
        self.get_logger().info(f'Prędkość sieczkarni: {self.chopper_speed_kmh:.1f} km/h ({self.chopper_speed_mps:.2f} m/s)')
        self.get_logger().info(f'Początkowa odległość wzdłużna: {self.initial_distance_m:.1f} m (0 = ta sama pozycja)')
        self.get_logger().info(f'Zakres odległości wzdłużnej: {self.min_distance_m:.1f} - {self.max_distance_m:.1f} m (ujemna = sieczkarnia przed ciągnikiem)')
        self.get_logger().info(f'Odległość poprzeczna: {self.lateral_distance_m:.1f} m (±{self.lateral_noise_std*1000:.0f} mm)')
        self.get_logger().info(f'Szum prędkości GPS: ±{self.gps_speed_noise_std:.3f} m/s')
        self.get_logger().info(f'Szum pozycji GPS: ±{self.gps_position_noise_std:.1f} m')
        self.get_logger().info('====================================================')
        
    def tractor_gps_callback(self, msg: GpsRtk):
        """Callback dla danych GPS ciągnika"""
        current_time = time.time()
        
        
        
        # Oblicz rzeczywisty dt
        if hasattr(self, 'last_callback_time'):
            dt = current_time - self.last_callback_time
        else:
            dt = 0.05  # Pierwszy callback
            # Ustaw prędkość sieczkarni na prędkość ciągnika przy pierwszym callbacku
            self.chopper_speed_mps = msg.speed_mps
            self.get_logger().info(f'Pierwszy callback GPS ciągnika - prędkość: {msg.speed_mps:.2f} m/s')
            self.get_logger().info(f'Ustawiono prędkość sieczkarni na: {self.chopper_speed_mps:.2f} m/s')
        self.last_callback_time = current_time
        
        self.last_tractor_position = msg
        self.last_tractor_speed = msg.speed_mps
        
        # Oblicz różnicę prędkości (użyj prędkości z szumem jeśli dostępna)
        chopper_speed = getattr(self, 'chopper_speed_with_noise', self.chopper_speed_mps)
        speed_diff = self.last_tractor_speed - chopper_speed
        
        # Całkowanie prędkości (odległość)
        self.current_distance_m += speed_diff * dt
        
        # Ograniczenia odległości
        self.current_distance_m = max(self.min_distance_m, 
                                    min(self.max_distance_m, self.current_distance_m))
        
        # Loguj zmiany odległości (co 2 sekundy)
        if not hasattr(self, 'last_log_time') or current_time - self.last_log_time > 2.0:
            self.get_logger().info(
                f'Odległość wzdłużna: {self.current_distance_m:.1f} m | '
                f'Odległość poprzeczna: {self.lateral_distance_m:.1f} m (±{self.lateral_noise_std*1000:.0f} mm) | '
                f'Prędkość ciągnika: {self.last_tractor_speed:.2f} m/s | '
                f'Prędkość sieczkarni: {chopper_speed:.2f} m/s | '
                f'Różnica: {speed_diff:.2f} m/s | '
                f'dt: {dt:.3f}s | '
            )
            self.last_log_time = current_time
    
    def publish_chopper_data(self):
        """Publikuje dane GPS sieczkarni"""
        if self.last_tractor_position is None:
            return
        
        # Utwórz wiadomość GPS sieczkarni
        chopper_msg = GpsRtk()
        
        # Timestamp
        chopper_msg.header.stamp = self.get_clock().now().to_msg()
        chopper_msg.header.frame_id = 'chopper_gps'
        
        # Pozycja sieczkarni = pozycja ciągnika + przesunięcie
        # Przesunięcie wzdłużne (na północ) - z całkowania prędkości
        distance_north_m = self.current_distance_m
        
        # Przesunięcie poprzeczne (na wschód) - stałe 6m + szum
        distance_east_m = self.lateral_distance_m + random.gauss(0, self.lateral_noise_std)
        
        # Konwersja metrów na stopnie (przybliżona)
        lat_offset = distance_north_m / 111000.0  # ~111 km na stopień
        lon_offset = distance_east_m / (111000.0 * 0.6)  # ~66 km na stopień na szerokości geograficznej
        
        # Pozycja sieczkarni
        chopper_msg.latitude_deg = self.last_tractor_position.latitude_deg + lat_offset
        chopper_msg.longitude_deg = self.last_tractor_position.longitude_deg + lon_offset
        chopper_msg.altitude_m = self.last_tractor_position.altitude_m
        
        # Prędkość sieczkarni z szumem GPS
        chopper_speed_with_noise = self.chopper_speed_mps + random.gauss(0, self.gps_speed_noise_std)
        chopper_msg.speed_mps = max(0.0, chopper_speed_with_noise)  # Nie może być ujemna
        
        # Zapisz prędkość z szumem do użycia w callbacku
        self.chopper_speed_with_noise = chopper_msg.speed_mps
        
        # Kurs sieczkarni (prosto na północ)
        chopper_msg.heading_deg = 0.0
        
        # Status RTK
        chopper_msg.rtk_status = self.rtk_status
        
        # Dodaj szum pozycji
        chopper_msg.latitude_deg += random.gauss(0, self.gps_position_noise_std / 111000.0)
        chopper_msg.longitude_deg += random.gauss(0, self.gps_position_noise_std / (111000.0 * 0.6))
        
        # Publikuj dane GPS sieczkarni
        self.chopper_gps_publisher.publish(chopper_msg)
        
        # Publikuj metryki odległości
        self.publish_distance_metrics()
    
    def publish_distance_metrics(self):
        """Publikuje metryki odległości na /distance_metrics"""
        if self.last_tractor_position is None:
            return
        
        # Utwórz wiadomość DistanceMetrics
        metrics_msg = DistanceMetrics()
        
        # Timestamp
        metrics_msg.header.stamp = self.get_clock().now().to_msg()
        metrics_msg.header.frame_id = 'chopper_simulator'
        
        # Oblicz odległość w linii prostej
        distance_straight = (self.current_distance_m**2 + self.lateral_distance_m**2)**0.5
        
        # Ustaw metryki
        metrics_msg.distance_straight = distance_straight
        metrics_msg.distance_longitudinal = self.current_distance_m
        metrics_msg.distance_lateral = self.lateral_distance_m
        
        # Publikuj
        self.distance_metrics_publisher.publish(metrics_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = ChopperDistanceSimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymywanie symulatora sieczkarni...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
