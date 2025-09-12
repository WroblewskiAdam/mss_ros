#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_srvs.srv import SetBool
import time
import json
import psutil
from std_msgs.msg import String

from my_robot_interfaces.msg import Gear

class GearMockupNode(Node):
    """
    Węzeł symulujący dane biegów/półbiegów dla trybu mockup.
    Publikuje realistyczne dane na topiku /gears.
    
    Logika:
    - Po uruchomieniu: bieg 1, sprzęgło zwolnione
    - Zmiana biegów przez serwisy /gear_shift_up i /gear_shift_down
    - Opóźnienie 1 sekunda przy zmianie biegów
    - Biegi od 1 do 4
    """
    
    def __init__(self):
        super().__init__('gear_mockup_node')
        
        # Parametry
        self.declare_parameter('publish_frequency_hz', 10.0)
        self.declare_parameter('shift_delay_sec', 1.0)
        self.declare_parameter('initial_gear', 1)
        self.declare_parameter('max_gear', 4)
        self.declare_parameter('min_gear', 1)
        
        self.publish_frequency = self.get_parameter('publish_frequency_hz').get_parameter_value().double_value
        self.shift_delay = self.get_parameter('shift_delay_sec').get_parameter_value().double_value
        self.max_gear = self.get_parameter('max_gear').get_parameter_value().integer_value
        self.min_gear = self.get_parameter('min_gear').get_parameter_value().integer_value
        initial_gear = self.get_parameter('initial_gear').get_parameter_value().integer_value
        
        # Stan symulacji
        self.current_gear = initial_gear
        self.clutch_state = 0  # 0 = zwolnione, 1 = wciśnięte
        self.is_shifting = False
        self.shift_start_time = 0.0
        self.target_gear = initial_gear
        
        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher
        self.gear_publisher = self.create_publisher(Gear, '/gears', qos_profile)
        
        # Serwisy do zmiany biegów
        self.shift_up_service = self.create_service(SetBool, '/gear_shift_up', self.shift_up_callback)
        self.shift_down_service = self.create_service(SetBool, '/gear_shift_down', self.shift_down_callback)
        
        # Serwis do symulacji sprzęgła (opcjonalny)
        self.clutch_service = self.create_service(SetBool, '/gear_mockup/set_clutch', self.set_clutch_callback)
        
        # Timery
        self.publish_timer = self.create_timer(1.0 / self.publish_frequency, self.publish_gear_data)
        self.shift_timer = self.create_timer(0.1, self.shift_process_callback)  # 10 Hz dla płynnej zmiany
        
        # Health monitoring
        self.health_pub = self.create_publisher(String, '/mss/node_health/gear_mockup_node', qos_profile)
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        self.get_logger().info(f"Gear Mockup Node uruchomiony.")
        self.get_logger().info(f"Początkowy bieg: {self.current_gear}")
        self.get_logger().info(f"Opóźnienie zmiany: {self.shift_delay}s")
        self.get_logger().info(f"Zakres biegów: {self.min_gear}-{self.max_gear}")
        self.get_logger().info("Serwisy dostępne: /gear_shift_up, /gear_shift_down, /gear_mockup/set_clutch")
    
    def shift_up_callback(self, request, response):
        """Callback dla serwisu zmiany biegu w górę."""
        if not request.data:
            response.success = False
            response.message = "Serwis wymaga request.data = true"
            return response
        
        if self.is_shifting:
            response.success = False
            response.message = f"Trwa zmiana biegu z {self.current_gear} na {self.target_gear}"
            return response
        
        if self.current_gear >= self.max_gear:
            response.success = False
            response.message = f"Już jesteś na najwyższym biegu ({self.max_gear})"
            return response
        
        # Rozpocznij zmianę biegu w górę
        self.target_gear = self.current_gear + 1
        self.start_shift()
        
        response.success = True
        response.message = f"Rozpoczęto zmianę biegu z {self.current_gear} na {self.target_gear}"
        self.get_logger().info(f"UPSHIFT: {self.current_gear} -> {self.target_gear}")
        
        return response
    
    def shift_down_callback(self, request, response):
        """Callback dla serwisu zmiany biegu w dół."""
        if not request.data:
            response.success = False
            response.message = "Serwis wymaga request.data = true"
            return response
        
        if self.is_shifting:
            response.success = False
            response.message = f"Trwa zmiana biegu z {self.current_gear} na {self.target_gear}"
            return response
        
        if self.current_gear <= self.min_gear:
            response.success = False
            response.message = f"Już jesteś na najniższym biegu ({self.min_gear})"
            return response
        
        # Rozpocznij zmianę biegu w dół
        self.target_gear = self.current_gear - 1
        self.start_shift()
        
        response.success = True
        response.message = f"Rozpoczęto zmianę biegu z {self.current_gear} na {self.target_gear}"
        self.get_logger().info(f"DOWNSHIFT: {self.current_gear} -> {self.target_gear}")
        
        return response
    
    def set_clutch_callback(self, request, response):
        """Callback dla serwisu ustawiania sprzęgła."""
        old_clutch = self.clutch_state
        self.clutch_state = 1 if request.data else 0
        
        clutch_text = "wciśnięte" if self.clutch_state == 1 else "zwolnione"
        response.success = True
        response.message = f"Sprzęgło {clutch_text}"
        
        if old_clutch != self.clutch_state:
            self.get_logger().info(f"Sprzęgło: {clutch_text}")
        
        return response
    
    def start_shift(self):
        """Rozpoczyna proces zmiany biegu."""
        self.is_shifting = True
        self.shift_start_time = time.time()
        self.clutch_state = 1  # Wciśnij sprzęgło podczas zmiany
        
        self.get_logger().info(f"Rozpoczęto zmianę biegu: {self.current_gear} -> {self.target_gear}")
        self.get_logger().info(f"Sprzęgło wciśnięte, oczekiwanie {self.shift_delay}s...")
    
    def shift_process_callback(self):
        """Callback timera obsługującego proces zmiany biegu."""
        if not self.is_shifting:
            return
        
        current_time = time.time()
        elapsed_time = current_time - self.shift_start_time
        
        if elapsed_time >= self.shift_delay:
            # Zakończ zmianę biegu
            self.current_gear = self.target_gear
            self.is_shifting = False
            self.clutch_state = 0  # Zwolnij sprzęgło
            
            self.get_logger().info(f"Zmiana biegu zakończona: bieg {self.current_gear}")
            self.get_logger().info("Sprzęgło zwolnione")
    
    def publish_gear_data(self):
        """Publikuje aktualne dane biegów."""
        msg = Gear()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gear_mockup"
        msg.gear = self.current_gear
        msg.clutch_state = self.clutch_state
        
        self.gear_publisher.publish(msg)
    
    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status publisherów
            publisher_status = "OK" if hasattr(self, 'gear_publisher') else "ERROR"
            
            # Sprawdź status serwisów
            services_status = "OK"
            if not hasattr(self, 'shift_up_service') or not hasattr(self, 'shift_down_service'):
                services_status = "ERROR"
            
            # Sprawdź status timerów
            timers_status = "OK"
            if not hasattr(self, 'publish_timer') or not hasattr(self, 'shift_timer'):
                timers_status = "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if publisher_status == "ERROR":
                errors.append("Publisher nieaktywny")
            if services_status == "ERROR":
                errors.append("Serwisy nieaktywne")
            if timers_status == "ERROR":
                errors.append("Timery nieaktywne")
            
            if self.is_shifting:
                warnings.append(f"Trwa zmiana biegu: {self.current_gear} -> {self.target_gear}")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'publisher_status': publisher_status,
                'services_status': services_status,
                'timers_status': timers_status,
                'current_gear': self.current_gear,
                'target_gear': self.target_gear,
                'clutch_state': self.clutch_state,
                'is_shifting': self.is_shifting,
                'shift_delay_sec': self.shift_delay,
                'gear_range': f"{self.min_gear}-{self.max_gear}",
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
    node = GearMockupNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Przerwano przez użytkownika (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
