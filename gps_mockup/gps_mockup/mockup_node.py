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
        self.declare_parameter('tractor_speed_mps', 2.0)  # 2 m/s = ~7.2 km/h
        self.declare_parameter('chopper_speed_mps', 1.9)  # 1.9 m/s = ~6.8 km/h (nieco wolniejsza)
        self.declare_parameter('chopper_offset_m', 5.0)   # 5m za ciągnikiem
        self.declare_parameter('simulation_area_lat', 52.2297)  # Warszawa
        self.declare_parameter('simulation_area_lon', 21.0122)
        
        self.publish_frequency = self.get_parameter('publish_frequency_hz').get_parameter_value().double_value
        self.tractor_speed = self.get_parameter('tractor_speed_mps').get_parameter_value().double_value
        self.chopper_speed = self.get_parameter('chopper_speed_mps').get_parameter_value().double_value
        self.chopper_offset = self.get_parameter('chopper_offset_m').get_parameter_value().double_value
        self.base_lat = self.get_parameter('simulation_area_lat').get_parameter_value().double_value
        self.base_lon = self.get_parameter('simulation_area_lon').get_parameter_value().double_value
        
        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
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
    
    def parameters_callback(self, params):
        """Callback do aktualizacji parametrów w czasie rzeczywistym"""
        for param in params:
            if param.name == 'tractor_speed_mps':
                self.tractor_speed = param.value
                self.get_logger().info(f'Zaktualizowano prędkość ciągnika: {self.tractor_speed} m/s')
            elif param.name == 'chopper_speed_mps':
                self.chopper_speed = param.value
                self.get_logger().info(f'Zaktualizowano prędkość sieczkarni: {self.chopper_speed} m/s')
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
        
        return SetParametersResult(successful=True)
    
    def publish_gps_data(self):
        """Publikuje dane GPS dla ciągnika i sieczkarni"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Aktualizuj pozycję ciągnika (prosty ruch w linii prostej z lekkimi zakrętami)
        self.update_tractor_position(elapsed_time)
        
        # Oblicz pozycję sieczkarni (za ciągnikiem)
        self.update_chopper_position()
        
        # Publikuj dane ciągnika
        tractor_msg = self.create_gps_message(
            self.tractor_position, 
            self.current_heading, 
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
    
    def update_tractor_position(self, elapsed_time):
        """Aktualizuje pozycję ciągnika z realistycznym ruchem"""
        # Symuluj ruch w linii prostej z lekkimi zakrętami
        # Ciągnik jedzie głównie na północ z lekkimi odchyleniami
        
        # Podstawowy ruch na północ
        distance_north = self.tractor_speed * elapsed_time
        
        # Dodaj sinusoidalne odchylenia (symulacja nierówności terenu)
        lateral_offset = 2.0 * math.sin(elapsed_time * 0.1)  # ±2m odchylenie
        
        # Okresowe zakręty
        heading_change = 5.0 * math.sin(elapsed_time * 0.05)  # ±5° zakręty
        self.current_heading = 0.0 + heading_change  # Głównie na północ
        
        # Konwertuj na współrzędne kartezjańskie
        heading_rad = math.radians(self.current_heading)
        self.tractor_position[0] = distance_north * math.cos(heading_rad) + lateral_offset
        self.tractor_position[1] = distance_north * math.sin(heading_rad)
    
    def update_chopper_position(self):
        """Oblicza pozycję sieczkarni względem ciągnika"""
        # Sieczkarnia jedzie za ciągnikiem w odległości chopper_offset
        heading_rad = math.radians(self.current_heading)
        
        # Pozycja za ciągnikiem
        self.chopper_position[0] = self.tractor_position[0] - self.chopper_offset * math.cos(heading_rad)
        self.chopper_position[1] = self.tractor_position[1] - self.chopper_offset * math.sin(heading_rad)
    
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
        
        # Dodaj niewielki szum do symulacji rzeczywistych warunków
        msg.latitude_deg += (random.random() - 0.5) * 0.00001  # ±0.5m
        msg.longitude_deg += (random.random() - 0.5) * 0.00001
        msg.speed_mps += (random.random() - 0.5) * 0.1  # ±0.05 m/s
        
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