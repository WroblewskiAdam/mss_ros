#!/usr/bin/env python3
"""
PRBS Generator Node - Generator sygnału PRBS do identyfikacji obiektu regulacji pozycji
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time

class PRBSGeneratorNode(Node):
    def __init__(self):
        super().__init__('prbs_generator_node')
        
        # --- PARAMETRY SYGNAŁU (w km/h, przeliczane do m/s) ---
        self.base_speed_kmh = 8.0   # Prędkość bazowa w km/h
        self.amplitude_kmh = 1.0    # Amplituda skoków w km/h (±1 km/h)
        self.min_interval_sec = 4.0 # Minimalny czas trwania jednego poziomu prędkości
        self.max_interval_sec = 10.0 # Maksymalny czas trwania
        self.publish_freq_hz = 20.0 # Częstotliwość publikacji
        
        # Przeliczenie km/h na m/s
        self.base_speed_mps = self.base_speed_kmh / 3.6
        self.amplitude_mps = self.amplitude_kmh / 3.6
        
        # --- PUBLISHER I TIMER ---
        self.publisher_ = self.create_publisher(Float64, '/target_speed', 10)
        self.timer = self.create_timer(1.0 / self.publish_freq_hz, self.publish_callback)
        
        # --- STAN WĘZŁA ---
        self.current_speed = self.base_speed_mps
        self.time_of_next_switch = time.time() + self.get_random_interval()
        
        # --- LOGOWANIE ---
        self.get_logger().info('=== PRBS GENERATOR NODE URUCHOMIONY ===')
        self.get_logger().info(f'Prędkość bazowa: {self.base_speed_kmh:.1f} km/h ({self.base_speed_mps:.2f} m/s)')
        self.get_logger().info(f'Amplituda: ±{self.amplitude_kmh:.1f} km/h (±{self.amplitude_mps:.2f} m/s)')
        self.get_logger().info(f'Zakres prędkości: {self.base_speed_kmh-self.amplitude_kmh:.1f} - {self.base_speed_kmh+self.amplitude_kmh:.1f} km/h')
        self.get_logger().info(f'Czas przełączeń: {self.min_interval_sec} - {self.max_interval_sec} sekund')
        self.get_logger().info(f'Częstotliwość publikacji: {self.publish_freq_hz} Hz')
        self.get_logger().info('Aby zakończyć, naciśnij Ctrl+C.')
        self.get_logger().info('==========================================')
        
    def get_random_interval(self):
        """Generuje losowy czas do następnego przełączenia."""
        return random.uniform(self.min_interval_sec, self.max_interval_sec)
    
    def publish_callback(self):
        """Główna funkcja publikująca sygnał PRBS."""
        current_time = time.time()
        
        # Sprawdź czy czas na przełączenie
        if current_time >= self.time_of_next_switch:
            # Przełącz prędkość
            if self.current_speed > self.base_speed_mps:
                self.current_speed = self.base_speed_mps - self.amplitude_mps
                direction = "↓"
            else:
                self.current_speed = self.base_speed_mps + self.amplitude_mps
                direction = "↑"
            
            # Ustaw czas następnego przełączenia
            self.time_of_next_switch = current_time + self.get_random_interval()
            
            # Loguj zmianę
            self.get_logger().info(
                f'PRBS: {direction} Zmiana prędkości zadanej na: '
                f'{self.current_speed*3.6:.1f} km/h ({self.current_speed:.2f} m/s)'
            )
        
        # Publikuj aktualną prędkość
        msg = Float64()
        msg.data = self.current_speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PRBSGeneratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymywanie generatora PRBS...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
