#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from my_robot_interfaces.msg import GpsRtk, Gear
import time
import json

class GearManagerNode(Node):
    def __init__(self):
        super().__init__('gear_manager_node')
        
        # --- Parametry ---
        self.declare_parameter('powershift_max_speeds_kmh', [8.4, 13.0, 16.8, 25.2])  # km/h
        self.declare_parameter('upshift_threshold_percent', 0.95)
        self.declare_parameter('downshift_threshold_percent', 0.85)
        self.declare_parameter('shift_cooldown_sec', 2.0)
        self.declare_parameter('max_powershift', 4)
        
        # Pobierz parametry
        powershift_max_speeds_kmh = self.get_parameter('powershift_max_speeds_kmh').get_parameter_value().double_array_value
        self.upshift_thresh_percent = self.get_parameter('upshift_threshold_percent').get_parameter_value().double_value
        self.downshift_thresh_percent = self.get_parameter('downshift_threshold_percent').get_parameter_value().double_value
        self.shift_cooldown = self.get_parameter('shift_cooldown_sec').get_parameter_value().double_value
        self.max_powershift = self.get_parameter('max_powershift').get_parameter_value().integer_value
        
        # Konwersja km/h na m/s
        self.powershift_max_speeds = [speed / 3.6 for speed in powershift_max_speeds_kmh]
        
        # --- Zmienne stanu ---
        self.current_powershift = 0  # Aktualny półbieg (0 = nieznany)
        self.current_speed = 0.0     # Aktualna prędkość w m/s
        self.clutch_pressed = False  # Stan sprzęgła
        self.last_shift_time = self.get_clock().now()  # Czas ostatniej zmiany
        self.powershift_initialized = False  # Czy półbieg został odczytany przy starcie
        self.is_enabled = True  # Stan włączania/wyłączania gear managera
        
        # --- Health monitoring ---
        self.shift_attempts = 0
        self.failed_shifts = 0
        self.last_gear_update = time.time()
        self.last_speed_update = time.time()
        
        # --- Subskrypcje ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subskrypcja stanu biegów
        self.gear_subscription = self.create_subscription(
            Gear,
            '/gears',
            self.gear_callback,
            qos_profile
        )
        
        # Subskrypcja prędkości GPS
        self.speed_subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data_filtered',
            self.speed_callback,
            qos_profile
        )
        
        # --- Klienci usług ---
        self.shift_up_client = self.create_client(SetBool, '/gear_shift_up')
        self.shift_down_client = self.create_client(SetBool, '/gear_shift_down')
        
        # --- Serwisy ---
        self.set_enabled_service = self.create_service(SetBool, '/gear_manager/set_enabled', self.set_enabled_callback)
        
        # --- Publisher health ---
        self.health_pub = self.create_publisher(String, '/mss/node_health/gear_manager_node', 10)
        
        # --- Timery ---
        self.decision_timer = self.create_timer(0.1, self.decision_loop)  # 10 Hz
        self.health_timer = self.create_timer(5.0, self.publish_health)  # Health co 5s
        
        # --- Logowanie ---
        self.get_logger().info("=== GEAR MANAGER NODE STARTED ===")
        self.get_logger().info(f"Maksymalne prędkości półbiegów: {powershift_max_speeds_kmh} km/h")
        self.get_logger().info(f"Progi: upshift={self.upshift_thresh_percent*100}%, downshift={self.downshift_thresh_percent*100}%")
        self.get_logger().info(f"Cooldown: {self.shift_cooldown}s")
        self.get_logger().info("Oczekiwanie na odczyt aktualnego półbiegu z topiku /gears...")

    def gear_callback(self, msg):
        """Odbiera stan biegu (półbiegu) i sprzęgła."""
        self.last_gear_update = time.time()
        
        # Debug: loguj tylko zmiany biegu
        if self.current_powershift != msg.gear:
            self.get_logger().info(f"DEBUG: Otrzymano z /gears - gear={msg.gear}, clutch={msg.clutch_state}, current_powershift={self.current_powershift}")
        
        if not self.powershift_initialized and msg.gear > 0:
            # Pierwszy odczyt - zainicjalizuj półbieg
            self.current_powershift = msg.gear
            self.powershift_initialized = True
            self.get_logger().info(f"Zainicjalizowano półbieg: {self.current_powershift}")
        elif self.current_powershift != msg.gear and msg.gear > 0:
            # Zmiana półbiegu
            old_powershift = self.current_powershift
            self.current_powershift = msg.gear
            self.get_logger().info(f"Zmiana półbiegu: {old_powershift} -> {self.current_powershift}")
        elif msg.gear > 0:
            # Aktualizacja stanu biegu (bez zmiany) - ważne dla synchronizacji
            if self.current_powershift != msg.gear:
                self.get_logger().info(f"Synchronizacja stanu biegu: {self.current_powershift} -> {msg.gear}")
            self.current_powershift = msg.gear
        
        # Aktualizuj stan sprzęgła
        self.clutch_pressed = (msg.clutch_state == 1)

    def speed_callback(self, msg):
        """Odbiera aktualną prędkość z GPS (filtrowaną)."""
        self.current_speed = msg.speed_mps
        self.last_speed_update = time.time()

    def call_shift_service(self, client, direction):
        """Wywołuje usługę zmiany biegu."""
        self.shift_attempts += 1
        
        if not client.service_is_ready():
            self.get_logger().error(f"Serwis {direction} nie jest dostępny")
            self.failed_shifts += 1
            return False
        
        request = SetBool.Request()
        request.data = True
        
        try:
            # Wyślij żądanie bez czekania na odpowiedź
            future = client.call_async(request)
            self.get_logger().info(f"Wysłano żądanie zmiany półbiegu: {direction}")
            self.last_shift_time = self.get_clock().now()
            return True  # Zawsze zwracaj True - stan będzie zsynchronizowany przez /gears
                
        except Exception as e:
            self.get_logger().error(f"Błąd usługi {direction}: {e}")
            self.failed_shifts += 1
            return False

    def set_enabled_callback(self, request, response):
        """Callback dla serwisu włączania/wyłączania gear managera."""
        self.is_enabled = request.data
        status = "włączony" if self.is_enabled else "wyłączony"
        self.get_logger().info(f"Gear manager {status}")
        
        response.success = True
        response.message = f"Gear manager {status}"
        return response

    def decision_loop(self):
        """Główna pętla decyzyjna."""
        # Sprawdź czy gear manager jest włączony
        if not self.is_enabled:
            return
            
        # Sprawdź czy półbieg został zainicjalizowany
        if not self.powershift_initialized:
            return
        
        # Warunki bezpieczeństwa
        if self.clutch_pressed or self.current_powershift == 0 or self.current_speed == 0.0:
            return
        
        # Cooldown po ostatniej zmianie
        time_since_last_shift = (self.get_clock().now() - self.last_shift_time).nanoseconds / 1e9
        if time_since_last_shift < self.shift_cooldown:
            return
        
        current_gear_index = self.current_powershift - 1
        
        # --- LOGIKA ZMIANY BIEGU W GÓRĘ ---
        if self.current_powershift < self.max_powershift:
            current_gear_max_speed = self.powershift_max_speeds[current_gear_index]
            upshift_speed_trigger = current_gear_max_speed * self.upshift_thresh_percent
            
            if self.current_speed > upshift_speed_trigger:
                current_kmh = self.current_speed * 3.6
                trigger_kmh = upshift_speed_trigger * 3.6
                self.get_logger().info(f"UPSHIFT: {current_kmh:.1f} km/h > {trigger_kmh:.1f} km/h (bieg {self.current_powershift})")
                self.get_logger().info(f"DEBUG: Przed upshift - current_powershift={self.current_powershift}")
                if self.call_shift_service(self.shift_up_client, "GÓRA"):
                    # Sukces - czekaj na aktualizację z /gears
                    return
                else:
                    # Niepowodzenie - nie próbuj ponownie przez cooldown
                    self.last_shift_time = self.get_clock().now()
                    return
        
        # --- LOGIKA ZMIANY BIEGU W DÓŁ ---
        if self.current_powershift > 1:
            lower_gear_index = current_gear_index - 1
            lower_gear_max_speed = self.powershift_max_speeds[lower_gear_index]
            downshift_speed_trigger = lower_gear_max_speed * self.downshift_thresh_percent
            
            if self.current_speed < downshift_speed_trigger:
                current_kmh = self.current_speed * 3.6
                trigger_kmh = downshift_speed_trigger * 3.6
                self.get_logger().info(f"DOWNSHIFT: {current_kmh:.1f} km/h < {trigger_kmh:.1f} km/h (bieg {self.current_powershift - 1})")
                self.get_logger().info(f"DEBUG: Przed downshift - current_powershift={self.current_powershift}")
                if self.call_shift_service(self.shift_down_client, "DÓŁ"):
                    # Sukces - czekaj na aktualizację z /gears
                    return
                else:
                    # Niepowodzenie - nie próbuj ponownie przez cooldown
                    self.last_shift_time = self.get_clock().now()
                    return

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            current_time = time.time()
            
            # Sprawdź status klientów usług
            shift_up_status = "OK" if self.shift_up_client.service_is_ready() else "ERROR"
            shift_down_status = "OK" if self.shift_down_client.service_is_ready() else "ERROR"
            
            # Sprawdź status serwisów
            set_enabled_service_status = "OK" if hasattr(self, 'set_enabled_service') else "ERROR"
            
            # Sprawdź status timerów
            decision_timer_status = "OK" if hasattr(self, 'decision_timer') else "ERROR"
            health_timer_status = "OK" if hasattr(self, 'health_timer') else "ERROR"
            
            # Sprawdź status publisher'a
            health_pub_status = "OK" if hasattr(self, 'health_pub') else "ERROR"
            
            # Sprawdź świeżość danych
            gear_data_age = current_time - self.last_gear_update
            speed_data_age = current_time - self.last_speed_update
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if shift_up_status == "ERROR":
                errors.append("Serwis shift_up niedostępny")
            if shift_down_status == "ERROR":
                errors.append("Serwis shift_down niedostępny")
            if set_enabled_service_status == "ERROR":
                errors.append("Serwis set_enabled nieaktywny")
            if decision_timer_status == "ERROR":
                errors.append("Timer decyzyjny nieaktywny")
            if health_timer_status == "ERROR":
                errors.append("Timer health nieaktywny")
            if health_pub_status == "ERROR":
                errors.append("Publisher health nieaktywny")
            
            # Sprawdź timeouty danych
            if gear_data_age > 10.0:  # 10 sekund bez danych o biegach
                warnings.append(f"Brak danych o biegach przez {gear_data_age:.1f}s")
            if speed_data_age > 5.0:  # 5 sekund bez danych o prędkości
                warnings.append(f"Brak danych o prędkości przez {speed_data_age:.1f}s")
            
            # Sprawdź statystyki zmian biegów
            if self.shift_attempts > 0:
                failure_rate = (self.failed_shifts / self.shift_attempts) * 100
                if failure_rate > 50.0:
                    warnings.append(f"Wysoki wskaźnik niepowodzeń zmian biegów: {failure_rate:.1f}%")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': current_time,
                'shift_up_status': shift_up_status,
                'shift_down_status': shift_down_status,
                'set_enabled_service_status': set_enabled_service_status,
                'decision_timer_status': decision_timer_status,
                'health_timer_status': health_timer_status,
                'health_pub_status': health_pub_status,
                'current_powershift': self.current_powershift,
                'current_speed_kmh': self.current_speed * 3.6,
                'clutch_pressed': self.clutch_pressed,
                'powershift_initialized': self.powershift_initialized,
                'is_enabled': self.is_enabled,
                'data_freshness': {
                    'gear_data_age_sec': gear_data_age,
                    'speed_data_age_sec': speed_data_age
                },
                'shift_statistics': {
                    'total_attempts': self.shift_attempts,
                    'failed_shifts': self.failed_shifts,
                    'success_rate_percent': ((self.shift_attempts - self.failed_shifts) / max(1, self.shift_attempts)) * 100
                },
                'errors': errors,
                'warnings': warnings
            }
            
            # Opublikuj health status
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
            
            # Debug logging
            self.get_logger().info(f"📤 HEALTH PUBLISHED: {health_data['status']}")
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas publikowania health status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GearManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()