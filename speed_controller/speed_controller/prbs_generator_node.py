#!/usr/bin/env python3
"""
PRBS Generator Node - Generator sygnału PRBS do identyfikacji obiektu regulacji pozycji
Obsługuje dwa tryby: sygnał losowych skoków (Random Telegraph) i formalny PRBS (LFSR)
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
import random
import time

class PRBSGeneratorNode(Node):
    def __init__(self):
        super().__init__('prbs_generator_node')
        
        # --- PARAMETR TRYBU GENEROWANIA ---
        self.declare_parameter('signal_mode', 'true_prbs')  # 'random_telegraph' lub 'true_prbs'
        # self.declare_parameter('signal_mode', 'random_telegraph')  # 'random_telegraph' lub 'true_prbs'
        self.signal_mode = self.get_parameter('signal_mode').get_parameter_value().string_value
        
        # Callback do obsługi zmian parametru w trakcie działania
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # --- PARAMETRY SYGNAŁU (w km/h, przeliczane do m/s) ---
        self.base_speed_kmh = 7.0   # Prędkość bazowa w km/h
        self.amplitude_kmh = 1.0    # Amplituda skoków w km/h (±1 km/h)
        self.min_interval_sec = 4.0 # Minimalny czas trwania jednego poziomu prędkości (tylko dla random_telegraph)
        self.max_interval_sec = 10.0 # Maksymalny czas trwania (tylko dla random_telegraph)
        self.publish_freq_hz = 20.0 # Częstotliwość publikacji
        
        # --- PARAMETRY PRAWDZIWEGO PRBS ---
        self.clock_period_sec = 2.0  # Okres zegara PRBS (dla true_prbs)
        self.lfsr_state = 0b10110101  # Stan początkowy LFSR (8 bitów) - większy rejestr dla lepszej losowości
        self.num_bits = 8            # Długość rejestru LFSR - zwiększona do 8 bitów
        self.lfsr_taps = [8, 6, 5, 4]  # Odczepy dla maksymalnej długości sekwencji (2^8-1=255) - wielomian pierwotny
        
        # Przeliczenie km/h na m/s
        self.base_speed_mps = self.base_speed_kmh / 3.6
        self.amplitude_mps = self.amplitude_kmh / 3.6
        
        # --- PUBLISHER I TIMERY ---
        self.publisher_ = self.create_publisher(Float64, '/target_speed', 10)
        
        # Timer do publikacji prędkości (wysoka częstotliwość)
        self.publish_timer = self.create_timer(1.0 / self.publish_freq_hz, self.publish_current_speed)
        
        # Timer zegara PRBS (tylko dla trybu true_prbs)
        self.prbs_timer = None
        if self.signal_mode == 'true_prbs':
            self.prbs_timer = self.create_timer(self.clock_period_sec, self.update_prbs_state)
        
        # --- STAN WĘZŁA ---
        self.current_speed = self.base_speed_mps
        
        # Stan dla trybu random_telegraph
        self.time_of_next_switch = None
        if self.signal_mode == 'random_telegraph':
            self.time_of_next_switch = time.time() + self.get_random_interval()
        
        # --- LOGOWANIE ---
        self.get_logger().info('=== PRBS GENERATOR NODE URUCHOMIONY ===')
        self.get_logger().info(f'TRYB: {self.signal_mode.upper()}')
        self.get_logger().info(f'Prędkość bazowa: {self.base_speed_kmh:.1f} km/h ({self.base_speed_mps:.2f} m/s)')
        self.get_logger().info(f'Amplituda: ±{self.amplitude_kmh:.1f} km/h (±{self.amplitude_mps:.2f} m/s)')
        self.get_logger().info(f'Zakres prędkości: {self.base_speed_kmh-self.amplitude_kmh:.1f} - {self.base_speed_kmh+self.amplitude_kmh:.1f} km/h')
        self.get_logger().info(f'Częstotliwość publikacji: {self.publish_freq_hz} Hz')
        
        if self.signal_mode == 'random_telegraph':
            self.get_logger().info(f'Czas przełączeń: {self.min_interval_sec} - {self.max_interval_sec} sekund')
        elif self.signal_mode == 'true_prbs':
            self.get_logger().info(f'Okres zegara PRBS: {self.clock_period_sec} sekund')
            self.get_logger().info(f'LFSR: {self.num_bits} bitów, odczepy {self.lfsr_taps}, długość sekwencji: {2**self.num_bits-1}')
            self.get_logger().info(f'Stan początkowy LFSR: {self.lfsr_state:08b}')
        
        self.get_logger().info('Aby zakończyć, naciśnij Ctrl+C.')
        self.get_logger().info('==========================================')
    
    def parameter_callback(self, params):
        """Callback obsługujący zmiany parametrów w trakcie działania."""
        for param in params:
            if param.name == 'signal_mode':
                new_mode = param.value
                if new_mode not in ['random_telegraph', 'true_prbs']:
                    self.get_logger().error(f'Nieprawidłowy tryb: {new_mode}. Dozwolone: random_telegraph, true_prbs')
                    result = SetParametersResult()
                    result.successful = False
                    result.reason = f'Nieprawidłowy tryb: {new_mode}'
                    return result
                
                if new_mode != self.signal_mode:
                    self.get_logger().info(f'Zmiana trybu z {self.signal_mode} na {new_mode}')
                    self.switch_signal_mode(new_mode)
                    self.signal_mode = new_mode
                    
        result = SetParametersResult()
        result.successful = True
        return result
    
    def switch_signal_mode(self, new_mode):
        """Przełącza tryb generowania sygnału w trakcie działania."""
        # Zatrzymaj timer PRBS jeśli istnieje
        if self.prbs_timer is not None:
            self.prbs_timer.cancel()
            self.prbs_timer = None
            self.get_logger().info('Zatrzymano timer zegara PRBS')
        
        # Resetuj stan
        if new_mode == 'random_telegraph':
            self.time_of_next_switch = time.time() + self.get_random_interval()
            self.get_logger().info('Przełączono na tryb Random Telegraph')
        elif new_mode == 'true_prbs':
            self.prbs_timer = self.create_timer(self.clock_period_sec, self.update_prbs_state)
            self.get_logger().info('Przełączono na tryb True PRBS (LFSR)')
        
        # Loguj aktualny stan
        self.get_logger().info(f'TRYB: {new_mode.upper()}')
        if new_mode == 'random_telegraph':
            self.get_logger().info(f'Czas przełączeń: {self.min_interval_sec} - {self.max_interval_sec} sekund')
        elif new_mode == 'true_prbs':
            self.get_logger().info(f'Okres zegara PRBS: {self.clock_period_sec} sekund')
            self.get_logger().info(f'LFSR: {self.num_bits} bitów, odczepy {self.lfsr_taps}, długość sekwencji: {2**self.num_bits-1}')
        
    def get_random_interval(self):
        """Generuje losowy czas do następnego przełączenia (tylko dla random_telegraph)."""
        return random.uniform(self.min_interval_sec, self.max_interval_sec)
    
    def update_prbs_state(self):
        """Aktualizuje stan PRBS zgodnie z algorytmem LFSR (tylko dla true_prbs)."""
        # 1. Pobierz bit wyjściowy (najmłodszy bit)
        output_bit = self.lfsr_state & 1
        
        # 2. Oblicz bit sprzężenia zwrotnego (XOR na bitach z odczepów)
        feedback_bit = 0
        for tap in self.lfsr_taps:
            feedback_bit ^= (self.lfsr_state >> (tap - 1)) & 1
        
        # 3. Przesuń rejestr w prawo
        self.lfsr_state >>= 1
        
        # 4. Ustaw najstarszy bit na wartość sprzężenia zwrotnego
        self.lfsr_state |= (feedback_bit << (self.num_bits - 1))
        
        # 5. Dodatkowe mieszanie dla lepszej losowości - XOR z bitem środkowym
        middle_bit = (self.lfsr_state >> (self.num_bits // 2)) & 1
        output_bit ^= middle_bit
        
        # 6. Ustaw prędkość na podstawie bitu wyjściowego
        if output_bit == 1:
            self.current_speed = self.base_speed_mps + self.amplitude_mps
            direction = "↑"
        else:
            self.current_speed = self.base_speed_mps - self.amplitude_mps
            direction = "↓"
        
        # Debug: pokaż stan LFSR i bit wyjściowy
        self.get_logger().info(
            f'PRBS Zegar: {direction} Zmiana prędkości na: '
            f'{self.current_speed*3.6:.1f} km/h ({self.current_speed:.2f} m/s) | '
            f'LFSR: {self.lfsr_state:08b} | Bit: {output_bit}'
        )
    
    def publish_current_speed(self):
        """Publikuje aktualnie zadaną prędkość z dużą częstotliwością."""
        # Dla trybu random_telegraph sprawdź czy czas na przełączenie
        if self.signal_mode == 'random_telegraph' and self.time_of_next_switch is not None:
            current_time = time.time()
            
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
                    f'Random Telegraph: {direction} Zmiana prędkości zadanej na: '
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
