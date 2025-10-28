#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import json
import time
from datetime import datetime
from typing import Optional

from std_msgs.msg import Float64, String, Bool
from std_srvs.srv import SetBool
from my_robot_interfaces.msg import (
    GpsRtk, Gear, StampedInt32, DiagnosticData, 
    DistanceMetrics, SpeedControllerState
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MSSDataLoggerNode(Node):
    """
    Węzeł do logowania wszystkich danych z systemu MSS do pliku CSV.
    Zbiera dane z wszystkich komponentów systemu synchronizacji prędkości i pozycji.
    """
    
    def __init__(self):
        super().__init__('mss_data_logger_node')
        
        # --- Konfiguracja plików ---
        log_directory = os.path.join(os.path.expanduser('~'), 'mss_ros/src/logs_mss')
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        
        # Nie tworzymy pliku przy inicjalizacji - tylko przy włączeniu logowania
        self.log_path = None
        self.csv_file = None
        self.csv_writer = None
        
        # Nagłówki kolumn CSV
        self.csv_headers = [
            # Czas
            'ros_time_sec', 'ros_time_nsec', 'system_time',
            
            # Pozycja ciągnika (surowe i przefiltrowane)
            'tractor_lat_raw', 'tractor_lon_raw',
            'tractor_lat_filtered', 'tractor_lon_filtered',
            
            # Prędkość ciągnika (surowe i przefiltrowane)
            'tractor_speed_raw', 'tractor_speed_filtered',
            
            # Heading i status RTK ciągnika
            'tractor_heading', 'tractor_rtk_status',
            
            # Biegi i sprzęgło
            'tractor_gear', 'tractor_clutch_state',
            
            # Serwo
            'servo_target_positon','servo_position',
            
            # Regulatory
            'speed_controller_enabled', 'target_speed',
            'position_controller_enabled', 'target_position',
            
            # Pozycja względna
            'distance_longitudinal', 'distance_lateral', 'distance_straight',
            
            # Sieczkarnia (surowe i przefiltrowane)
            'chopper_lat_raw', 'chopper_lon_raw',
            'chopper_lat_filtered', 'chopper_lon_filtered',
            'chopper_speed_raw', 'chopper_speed_filtered',
            'chopper_heading', 'chopper_rtk_status',
            
            # Status systemu
            'bt_status', 'autopilot_status'
        ]
        
        # Nie zapisujemy nagłówków przy inicjalizacji - tylko przy włączeniu logowania
        self.get_logger().info("MSS Data Logger zainicjalizowany - oczekiwanie na włączenie logowania")
        
        # --- Zmienne do przechowywania ostatnich wiadomości ---
        self.last_tractor_gps_raw: Optional[GpsRtk] = None
        self.last_tractor_gps_filtered: Optional[GpsRtk] = None
        self.last_chopper_gps_raw: Optional[GpsRtk] = None
        self.last_chopper_gps_filtered: Optional[GpsRtk] = None
        self.last_servo_position: Optional[StampedInt32] = None
        self.last_servo_target_position: Optional[StampedInt32] = None
        self.last_gear: Optional[Gear] = None
        self.last_distance_metrics: Optional[DistanceMetrics] = None
        self.last_speed_controller_state: Optional[SpeedControllerState] = None
        self.last_target_speed: Optional[Float64] = None
        self.last_target_position: Optional[Float64] = None
        self.last_autopilot_status: Optional[String] = None
        
        # --- Placeholder values ---
        self.PLACEHOLDER_UINT8 = 255
        self.PLACEHOLDER_INT = 99999
        self.PLACEHOLDER_FLOAT = 99999.0
        
        # --- Stan logowania ---
        self.is_logging_enabled = False
        
        # --- QoS Profile ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- Subskrypcje ---
        # Dane z ciągnika (surowe)
        self.tractor_gps_raw_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/tractor',
            self.tractor_gps_raw_callback,
            qos_profile
        )
        
        # Dane z ciągnika (przefiltrowane)
        self.tractor_gps_filtered_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/tractor_filtered',
            self.tractor_gps_filtered_callback,
            qos_profile
        )
        
        # Dane z sieczkarni (surowe)
        self.chopper_gps_raw_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/chopper',
            self.chopper_gps_raw_callback,
            qos_profile
        )
        
        # Dane z sieczkarni (przefiltrowane)
        self.chopper_gps_filtered_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data/chopper_filtered',
            self.chopper_gps_filtered_callback,
            qos_profile
        )
        
        # Pozycja serwa
        self.servo_position_subscription = self.create_subscription(
            StampedInt32,
            '/servo/position',
            self.servo_position_callback,
            qos_profile
        )

         # Pozycja zadana serwa
        self.servo_target_position_subscription = self.create_subscription(
            StampedInt32,
            '/servo/set_angle',
            self.servo_target_position_callback,
            qos_profile
        )
        
        # Biegi i sprzęgło
        self.gear_subscription = self.create_subscription(
            Gear,
            '/gears',
            self.gear_callback,
            qos_profile
        )
        
        # Pozycja względna
        self.distance_metrics_subscription = self.create_subscription(
            DistanceMetrics,
            '/distance_metrics',
            self.distance_metrics_callback,
            qos_profile
        )
        
        # Stan regulatora prędkości
        self.speed_controller_state_subscription = self.create_subscription(
            SpeedControllerState,
            '/speed_controller/state',
            self.speed_controller_state_callback,
            qos_profile
        )
        
        # Prędkość zadana
        self.target_speed_subscription = self.create_subscription(
            Float64,
            '/target_speed',
            self.target_speed_callback,
            qos_profile
        )
        
        # Pozycja zadana
        self.target_position_subscription = self.create_subscription(
            Float64,
            '/target_position',
            self.target_position_callback,
            qos_profile
        )
        
        
        # Status autopilota
        self.autopilot_status_subscription = self.create_subscription(
            String,
            '/autopilot/status',
            self.autopilot_status_callback,
            qos_profile
        )
        
        # Timer do zapisu danych (10 Hz)
        self.log_timer = self.create_timer(0.1, self.log_data)
        
        # Serwis do włączania/wyłączania logowania
        self.set_enabled_service = self.create_service(
            SetBool,
            '/mss_data_logger/set_enabled',
            self.set_enabled_callback
        )
        
        # Licznik zapisów
        self.log_count = 0
        
        self.get_logger().info("MSS Data Logger zainicjalizowany - zbieranie danych z całego systemu")
    
    def tractor_gps_raw_callback(self, msg: GpsRtk):
        """Callback dla surowych danych GPS ciągnika"""
        self.last_tractor_gps_raw = msg
    
    def tractor_gps_filtered_callback(self, msg: GpsRtk):
        """Callback dla przefiltrowanych danych GPS ciągnika"""
        self.last_tractor_gps_filtered = msg
    
    def chopper_gps_raw_callback(self, msg: GpsRtk):
        """Callback dla surowych danych GPS sieczkarni"""
        self.last_chopper_gps_raw = msg
    
    def chopper_gps_filtered_callback(self, msg: GpsRtk):
        """Callback dla przefiltrowanych danych GPS sieczkarni"""
        self.last_chopper_gps_filtered = msg
    
    def servo_position_callback(self, msg: StampedInt32):
        """Callback dla pozycji serwa"""
        self.last_servo_position = msg

    def servo_target_position_callback(self, msg: StampedInt32):
        """Callback dla pozycji serwa"""
        self.last_servo_target_position = msg
    
    def gear_callback(self, msg: Gear):
        """Callback dla biegów i sprzęgła"""
        self.last_gear = msg
    
    def distance_metrics_callback(self, msg: DistanceMetrics):
        """Callback dla metryk odległości"""
        self.last_distance_metrics = msg
    
    def speed_controller_state_callback(self, msg: SpeedControllerState):
        """Callback dla stanu regulatora prędkości"""
        self.last_speed_controller_state = msg
    
    def target_speed_callback(self, msg: Float64):
        """Callback dla prędkości zadanej"""
        self.last_target_speed = msg
    
    def target_position_callback(self, msg: Float64):
        """Callback dla pozycji zadanej"""
        self.last_target_position = msg
    
    
    def autopilot_status_callback(self, msg: String):
        """Callback dla statusu autopilota"""
        self.last_autopilot_status = msg
    
    def set_enabled_callback(self, request, response):
        """Callback dla serwisu włączania/wyłączania logowania"""
        self.is_logging_enabled = request.data
        
        if self.is_logging_enabled:
            # Tworzenie nowego pliku przy każdym włączeniu
            log_directory = os.path.join(os.path.expanduser('~'), 'mss_ros/src/logs_mss')
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.log_path = os.path.join(log_directory, f'mss_system_log_{timestamp}.csv')
            
            # Zamknij stary plik jeśli istnieje
            if self.csv_file:
                self.csv_file.close()
            
            # Otwórz nowy plik i zapisz nagłówki
            self.csv_file = open(self.log_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.csv_headers)
            
            self.get_logger().info(f"Logowanie włączone - nowy plik: {self.log_path}")
        else:
            # Zamknij plik przy wyłączeniu
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                self.log_path = None
            
            self.get_logger().info("Logowanie wyłączone")
        
        response.success = True
        response.message = f"Logowanie {'włączone' if self.is_logging_enabled else 'wyłączone'}"
        return response
    
    def log_data(self):
        """Główna funkcja logowania danych - wywoływana przez timer"""
        # Loguj tylko gdy włączone
        if not self.is_logging_enabled:
            return
            
        # Przygotuj dane do zapisu
        current_time = time.time()
        
        # Użyj timestamp z ciągnika jeśli dostępny, w przeciwnym razie systemowy
        if self.last_tractor_gps_raw:
            timestamp = self.last_tractor_gps_raw.header.stamp
        elif self.last_tractor_gps_filtered:
            timestamp = self.last_tractor_gps_filtered.header.stamp
        else:
            # Fallback timestamp
            from builtin_interfaces.msg import Time
            timestamp = Time()
            timestamp.sec = int(current_time)
            timestamp.nanosec = int((current_time - int(current_time)) * 1e9)
        
        # Dane z ciągnika (surowe i przefiltrowane)
        if self.last_tractor_gps_raw:
            tractor_lat_raw = self.last_tractor_gps_raw.latitude_deg
            tractor_lon_raw = self.last_tractor_gps_raw.longitude_deg
            tractor_speed_raw = self.last_tractor_gps_raw.speed_mps
            tractor_heading_raw = self.last_tractor_gps_raw.heading_deg
            tractor_rtk_status_raw = self.last_tractor_gps_raw.rtk_status
        else:
            tractor_lat_raw = self.PLACEHOLDER_FLOAT
            tractor_lon_raw = self.PLACEHOLDER_FLOAT
            tractor_speed_raw = self.PLACEHOLDER_FLOAT
            tractor_heading_raw = self.PLACEHOLDER_FLOAT
            tractor_rtk_status_raw = self.PLACEHOLDER_UINT8
        
        if self.last_tractor_gps_filtered:
            tractor_lat_filtered = self.last_tractor_gps_filtered.latitude_deg
            tractor_lon_filtered = self.last_tractor_gps_filtered.longitude_deg
            tractor_speed_filtered = self.last_tractor_gps_filtered.speed_mps
            tractor_heading_filtered = self.last_tractor_gps_filtered.heading_deg
            tractor_rtk_status_filtered = self.last_tractor_gps_filtered.rtk_status
        else:
            tractor_lat_filtered = self.PLACEHOLDER_FLOAT
            tractor_lon_filtered = self.PLACEHOLDER_FLOAT
            tractor_speed_filtered = self.PLACEHOLDER_FLOAT
            tractor_heading_filtered = self.PLACEHOLDER_FLOAT
            tractor_rtk_status_filtered = self.PLACEHOLDER_UINT8
        
        # Dane z sieczkarni (surowe i przefiltrowane)
        if self.last_chopper_gps_raw:
            chopper_lat_raw = self.last_chopper_gps_raw.latitude_deg
            chopper_lon_raw = self.last_chopper_gps_raw.longitude_deg
            chopper_speed_raw = self.last_chopper_gps_raw.speed_mps
            chopper_heading_raw = self.last_chopper_gps_raw.heading_deg
            chopper_rtk_status_raw = self.last_chopper_gps_raw.rtk_status
            bt_status = True
        else:
            chopper_lat_raw = self.PLACEHOLDER_FLOAT
            chopper_lon_raw = self.PLACEHOLDER_FLOAT
            chopper_speed_raw = self.PLACEHOLDER_FLOAT
            chopper_heading_raw = self.PLACEHOLDER_FLOAT
            chopper_rtk_status_raw = self.PLACEHOLDER_UINT8
            bt_status = False
        
        if self.last_chopper_gps_filtered:
            chopper_lat_filtered = self.last_chopper_gps_filtered.latitude_deg
            chopper_lon_filtered = self.last_chopper_gps_filtered.longitude_deg
            chopper_speed_filtered = self.last_chopper_gps_filtered.speed_mps
            chopper_heading_filtered = self.last_chopper_gps_filtered.heading_deg
            chopper_rtk_status_filtered = self.last_chopper_gps_filtered.rtk_status
        else:
            chopper_lat_filtered = self.PLACEHOLDER_FLOAT
            chopper_lon_filtered = self.PLACEHOLDER_FLOAT
            chopper_speed_filtered = self.PLACEHOLDER_FLOAT
            chopper_heading_filtered = self.PLACEHOLDER_FLOAT
            chopper_rtk_status_filtered = self.PLACEHOLDER_UINT8
        
        # Dane z serwa (z placeholderami jeśli brak)
        if self.last_servo_position:
            servo_position = self.last_servo_position.data
        else:
            servo_position = self.PLACEHOLDER_INT
        
         # Dane z serwa (z placeholderami jeśli brak)
        if self.last_servo_target_position:
            servo_target_position = self.last_servo_target_position.data
        else:
            servo_target_position = self.PLACEHOLDER_INT
        
        # Dane z biegów (z placeholderami jeśli brak)
        if self.last_gear:
            tractor_gear = self.last_gear.gear
            tractor_clutch_state = self.last_gear.clutch_state
        else:
            tractor_gear = self.PLACEHOLDER_UINT8
            tractor_clutch_state = self.PLACEHOLDER_UINT8
        
        # Dane z metryk odległości
        distance_longitudinal = 0.0
        distance_lateral = 0.0
        distance_straight = 0.0
        if self.last_distance_metrics:
            distance_longitudinal = self.last_distance_metrics.distance_longitudinal
            distance_lateral = self.last_distance_metrics.distance_lateral
            distance_straight = self.last_distance_metrics.distance_straight
        
        # Stan regulatora prędkości
        speed_controller_enabled = 0
        if self.last_speed_controller_state:
            # Sprawdź czy regulator jest aktywny (na podstawie danych)
            speed_controller_enabled = 1 if self.last_speed_controller_state.control_output != 0.0 else 0
        
        # Prędkość zadana
        target_speed = 0.0
        if self.last_target_speed:
            target_speed = self.last_target_speed.data
        
        # Pozycja zadana
        target_position = 0.0
        if self.last_target_position:
            target_position = self.last_target_position.data
        
        # Stan regulatora pozycji (na podstawie obecności danych)
        position_controller_enabled = 1 if self.last_target_position else 0
        
        
        # Status autopilota
        autopilot_status = "unknown"
        if self.last_autopilot_status:
            autopilot_status = self.last_autopilot_status.data
        
        # Przygotuj wiersz danych
        row_data = [
            # Czas
            timestamp.sec, timestamp.nanosec, current_time,
            
            # Pozycja ciągnika (surowe i przefiltrowane)
            tractor_lat_raw, tractor_lon_raw,
            tractor_lat_filtered, tractor_lon_filtered,
            
            # Prędkość ciągnika (surowe i przefiltrowane)
            tractor_speed_raw, tractor_speed_filtered,
            
            # Heading i status RTK ciągnika (używamy przefiltrowanych)
            tractor_heading_filtered, tractor_rtk_status_filtered,
            
            # Biegi i sprzęgło
            tractor_gear, tractor_clutch_state,
            
            # Serwo
            servo_target_position, servo_position,
            
            # Regulatory
            speed_controller_enabled, target_speed,
            position_controller_enabled, target_position,
            
            # Pozycja względna
            distance_longitudinal, distance_lateral, distance_straight,
            
            # Sieczkarnia (surowe i przefiltrowane)
            chopper_lat_raw, chopper_lon_raw,
            chopper_lat_filtered, chopper_lon_filtered,
            chopper_speed_raw, chopper_speed_filtered,
            chopper_heading_filtered, chopper_rtk_status_filtered,
            
            # Status systemu
            1 if bt_status else 0, autopilot_status
        ]
        
        # Zapisz dane
        self.csv_writer.writerow(row_data)
        self.csv_file.flush()
        
        self.log_count += 1
        if self.log_count % 100 == 0:  # Loguj co 100 zapisów
            self.get_logger().info(f"Zapisano {self.log_count} rekordów do pliku logów")
    
    def destroy_node(self):
        """Zamknij plik logów przy zamykaniu węzła"""
        if self.is_logging_enabled and self.csv_file:
            self.get_logger().info(f'Zamykanie pliku logów. Łącznie zapisano {self.log_count} rekordów.')
            self.csv_file.close()
        else:
            self.get_logger().info('Zamykanie węzła - logowanie nie było aktywne.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mss_data_logger_node = None
    try:
        mss_data_logger_node = MSSDataLoggerNode()
        rclpy.spin(mss_data_logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        if mss_data_logger_node:
            mss_data_logger_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
