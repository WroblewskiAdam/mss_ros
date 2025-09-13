#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
import math
import random
import time
from datetime import datetime, timezone

from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import Header

class GpsMockupNode(Node):
    """
    Węzeł symulujący dane GPS dla ciągnika i sieczkarni.
    Publikuje realistyczne dane GPS RTK na topikach:
    - /gps_rtk_data (ciągnik)
    - /gps_rtk_data/chopper (sieczkarnia)
    """
    
    def __init__(self):
        super().__init__('gps_mockup_node')
        
        # Parametry
        self.declare_parameter('publish_frequency_hz', 10.0)
        self.declare_parameter('tractor_speed_kmh', 7.2)  # 7.2 km/h = ~2.0 m/s
        self.declare_parameter('chopper_speed_kmh', 6.8)  # 6.8 km/h = ~1.9 m/s (nieco wolniejsza)
        self.declare_parameter('chopper_offset_m', 5.0)   # 5m za ciągnikiem
        self.declare_parameter('simulation_area_lat', 52.2297)  # Warszawa
        self.declare_parameter('simulation_area_lon', 21.0122)
        
        # NOWE: Parametry do sterowania pozycją ciągnika względem sieczkarni
        self.declare_parameter('tractor_offset_longitudinal', 0.0)  # Wzdłuż (m) - dodatnie = przed sieczkarnią
        self.declare_parameter('tractor_offset_lateral', 0.0)       # Poprzecz (m) - dodatnie = na prawo od sieczkarni
        
        # NOWE: Parametr do sterowania kursem ciągnika
        self.declare_parameter('tractor_heading_offset_deg', 0.0)   # Offset kursu (stopnie) - dodatnie = w prawo, ujemne = w lewo
        
        self.publish_frequency = self.get_parameter('publish_frequency_hz').get_parameter_value().double_value
        
        # Konwersja z km/h na m/s
        tractor_speed_kmh = self.get_parameter('tractor_speed_kmh').get_parameter_value().double_value
        chopper_speed_kmh = self.get_parameter('chopper_speed_kmh').get_parameter_value().double_value
        self.tractor_speed = tractor_speed_kmh / 3.6  # km/h -> m/s
        self.chopper_speed = chopper_speed_kmh / 3.6  # km/h -> m/s
        self.chopper_offset = self.get_parameter('chopper_offset_m').get_parameter_value().double_value
        self.base_lat = self.get_parameter('simulation_area_lat').get_parameter_value().double_value
        self.base_lon = self.get_parameter('simulation_area_lon').get_parameter_value().double_value
        
        # NOWE: Parametry pozycji ciągnika względem sieczkarni
        self.tractor_offset_longitudinal = self.get_parameter('tractor_offset_longitudinal').get_parameter_value().double_value
        self.tractor_offset_lateral = self.get_parameter('tractor_offset_lateral').get_parameter_value().double_value
        self.tractor_heading_offset = self.get_parameter('tractor_heading_offset_deg').get_parameter_value().double_value
        
        # QoS - RELIABLE dla kompatybilności z innymi węzłami
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishery
        self.tractor_publisher = self.create_publisher(
            GpsRtk, '/gps_rtk_data', qos_profile
        )
        self.chopper_publisher = self.create_publisher(
            GpsRtk, '/gps_rtk_data/chopper', qos_profile
        )
        
        # Stan symulacji
        self.start_time = time.time()
        self.current_heading = 0.0  # Kierunek w stopniach
        self.tractor_position = [0.0, 0.0]  # [x, y] w metrach względem punktu bazowego
        self.chopper_position = [0.0, 0.0]
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_gps_data)
        
        # Callback do parametrów - aktualizuje wartości w czasie rzeczywistym
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info(f'GPS Mockup Node uruchomiony')
        self.get_logger().info(f'Częstotliwość: {self.publish_frequency} Hz')
        self.get_logger().info(f'Prędkość ciągnika: {self.tractor_speed} m/s')
        self.get_logger().info(f'Prędkość sieczkarni: {self.chopper_speed} m/s')
        self.get_logger().info(f'Offset sieczkarni: {self.chopper_offset} m')
        self.get_logger().info(f'Punkt bazowy: {self.base_lat}, {self.base_lon}')
        self.get_logger().info(f'Offset wzdłużny ciągnika: {self.tractor_offset_longitudinal} m')
        self.get_logger().info(f'Offset poprzeczny ciągnika: {self.tractor_offset_lateral} m')
        self.get_logger().info(f'Offset kursu ciągnika: {self.tractor_heading_offset}°')
        self.get_logger().info(f'Szum GPS RTK FIX: pozycja ±1.5cm, prędkość ±0.015 m/s')
    
    def parameters_callback(self, params):
        """Callback do aktualizacji parametrów w czasie rzeczywistym"""
        for param in params:
            if param.name == 'tractor_speed_kmh':
                self.tractor_speed = param.value / 3.6  # km/h -> m/s
                self.get_logger().info(f'Zaktualizowano prędkość ciągnika: {param.value} km/h ({self.tractor_speed:.2f} m/s)')
            elif param.name == 'chopper_speed_kmh':
                self.chopper_speed = param.value / 3.6  # km/h -> m/s
                self.get_logger().info(f'Zaktualizowano prędkość sieczkarni: {param.value} km/h ({self.chopper_speed:.2f} m/s)')
            elif param.name == 'chopper_offset_m':
                self.chopper_offset = param.value
                self.get_logger().info(f'Zaktualizowano offset sieczkarni: {self.chopper_offset} m')
            elif param.name == 'publish_frequency_hz':
                self.publish_frequency = param.value
                # Restart timer z nową częstotliwością
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_gps_data)
                self.get_logger().info(f'Zaktualizowano częstotliwość: {self.publish_frequency} Hz')
            elif param.name == 'simulation_area_lat':
                self.base_lat = param.value
                self.get_logger().info(f'Zaktualizowano szerokość geograficzną: {self.base_lat}')
            elif param.name == 'simulation_area_lon':
                self.base_lon = param.value
                self.get_logger().info(f'Zaktualizowano długość geograficzną: {self.base_lon}')
            elif param.name == 'tractor_offset_longitudinal':
                self.tractor_offset_longitudinal = param.value
                self.get_logger().info(f'Zaktualizowano offset wzdłużny ciągnika: {self.tractor_offset_longitudinal} m')
            elif param.name == 'tractor_offset_lateral':
                self.tractor_offset_lateral = param.value
                self.get_logger().info(f'Zaktualizowano offset poprzeczny ciągnika: {self.tractor_offset_lateral} m')
            elif param.name == 'tractor_heading_offset_deg':
                self.tractor_heading_offset = param.value
                self.get_logger().info(f'Zaktualizowano offset kursu ciągnika: {self.tractor_heading_offset}°')
        
        return SetParametersResult(successful=True)
    
    def publish_gps_data(self):
        """Publikuje dane GPS dla ciągnika i sieczkarni"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # ZMIANA: Sieczkarnia jako punkt centralny - aktualizuj pozycję sieczkarni
        self.update_chopper_position(elapsed_time)
        
        # Oblicz pozycję ciągnika względem sieczkarni (z offsetami)
        self.update_tractor_position()
        
        # Publikuj dane ciągnika (z offsetem kursu)
        tractor_heading = self.current_heading + self.tractor_heading_offset
        tractor_msg = self.create_gps_message(
            self.tractor_position, 
            tractor_heading, 
            self.tractor_speed,
            "TRACTOR"
        )
        self.tractor_publisher.publish(tractor_msg)
        
        # Publikuj dane sieczkarni
        chopper_msg = self.create_gps_message(
            self.chopper_position, 
            self.current_heading, 
            self.chopper_speed,  # Użyj konfigurowalnej prędkości sieczkarni
            "CHOPPER"
        )
        self.chopper_publisher.publish(chopper_msg)
    
    def update_chopper_position(self, elapsed_time):
        """Aktualizuje pozycję sieczkarni (punkt centralny) z realistycznym ruchem"""
        # Symuluj ruch w linii prostej z lekkimi zakrętami
        # Sieczkarnia jedzie głównie na północ z lekkimi odchyleniami
        
        # Podstawowy ruch na północ
        distance_north = self.chopper_speed * elapsed_time
        
        # Dodaj sinusoidalne odchylenia (symulacja nierówności terenu)
        lateral_offset = 1.5 * math.sin(elapsed_time * 0.1)  # ±1.5m odchylenie
        
        # Okresowe zakręty
        heading_change = 3.0 * math.sin(elapsed_time * 0.05)  # ±3° zakręty
        self.current_heading = 0.0 + heading_change  # Głównie na północ
        
        # Konwertuj na współrzędne kartezjańskie
        heading_rad = math.radians(self.current_heading)
        self.chopper_position[0] = distance_north * math.cos(heading_rad) + lateral_offset
        self.chopper_position[1] = distance_north * math.sin(heading_rad)
    
    def update_tractor_position(self):
        """Oblicza pozycję ciągnika względem sieczkarni (punkt centralny)"""
        # Ciągnik jest pozycjonowany względem sieczkarni z konfigurowalnymi offsetami
        heading_rad = math.radians(self.current_heading)
        
        # POPRAWKA: Dostosowanie do systemu ENU używającego w relative_computer_node
        # W systemie ENU: X = East, Y = North
        # heading_vector = [sin(heading), cos(heading)] = [East, North]
        # 
        # Pozycja ciągnika względem sieczkarni:
        # - offset_longitudinal: wzdłuż osi jazdy sieczkarni (dodatnie = przed sieczkarnią)
        # - offset_lateral: poprzecznie do osi jazdy sieczkarni (dodatnie = na prawo od sieczkarni)
        
        # Współrzędne ENU: [X=East, Y=North]
        # heading_vector = [sin(heading), cos(heading)] = [East, North]
        east_offset = self.tractor_offset_longitudinal * math.sin(heading_rad) + self.tractor_offset_lateral * math.cos(heading_rad)
        north_offset = self.tractor_offset_longitudinal * math.cos(heading_rad) - self.tractor_offset_lateral * math.sin(heading_rad)
        
        self.tractor_position[0] = self.chopper_position[0] + east_offset   # X = East
        self.tractor_position[1] = self.chopper_position[1] + north_offset  # Y = North
    
    def create_gps_message(self, position, heading, speed, vehicle_type):
        """Tworzy wiadomość GPS RTK"""
        msg = GpsRtk()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'gps_{vehicle_type.lower()}'
        
        # Konwertuj pozycję kartezjańską na współrzędne geograficzne
        lat, lon = self.cartesian_to_gps(position[0], position[1])
        
        # Dane GPS
        msg.latitude_deg = lat
        msg.longitude_deg = lon
        msg.altitude_m = 120.0 + (2.0 * math.sin(time.time() * 0.1))  # 120m ±2m
        
        # Kurs i prędkość
        msg.heading_deg = heading
        msg.speed_mps = speed
        
        # Status RTK (symuluj różne stany)
        rtk_statuses = [1, 2, 4, 5]  # SPS, DGPS, FIX, FLOAT
        msg.rtk_status = rtk_statuses[int(time.time()) % len(rtk_statuses)]
        
        # Czas GPS
        msg.gps_time.sec = int(time.time())
        msg.gps_time.nanosec = int((time.time() % 1) * 1e9)
        
        # Dodaj realistyczny szum GPS RTK FIX
        # RTK FIX: pozycja ±1-2cm, prędkość ±0.01-0.02 m/s
        position_noise_std = 0.015  # 1.5cm standard deviation
        velocity_noise_std = 0.015  # 0.015 m/s standard deviation
        
        # Szum pozycji (Gaussian noise)
        lat_noise = random.gauss(0, position_noise_std / 111320.0)  # Konwersja na stopnie
        lon_noise = random.gauss(0, position_noise_std / (111320.0 * math.cos(math.radians(lat))))
        
        msg.latitude_deg += lat_noise
        msg.longitude_deg += lon_noise
        
        # Szum prędkości (Gaussian noise)
        speed_noise = random.gauss(0, velocity_noise_std)
        msg.speed_mps += speed_noise
        
        # Szum wysokości (RTK FIX: ±2-3cm)
        altitude_noise = random.gauss(0, 0.025)  # 2.5cm standard deviation
        msg.altitude_m += altitude_noise
        
        # Szum kursu (RTK FIX: ±0.1-0.2°)
        heading_noise = random.gauss(0, 0.15)  # 0.15° standard deviation
        msg.heading_deg += heading_noise
        
        return msg
    
    def cartesian_to_gps(self, x, y):
        """Konwertuje współrzędne kartezjańskie na geograficzne"""
        # Prosta konwersja (dla małych odległości)
        # 1 stopień ≈ 111,320 m na równoleżniku
        lat_offset = y / 111320.0
        lon_offset = x / (111320.0 * math.cos(math.radians(self.base_lat)))
        
        return self.base_lat + lat_offset, self.base_lon + lon_offset


def main(args=None):
    rclpy.init(args=args)
    
    node = GpsMockupNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
