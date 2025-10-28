#!/usr/bin/env python3
"""
Generator Sygnału Losowego (Random Speed Level Generator)

Wersja zmodyfikowana: Prędkość Bazowa +/- Losowa Zmiana

Generuje sygnał prędkości oparty na prędkości bazowej.
W losowych odstępach czasu (będących wielokrotnością 'interval_dt'
i mieszczących się w zakresie 'min/max_interval_sec')
sygnał przełącza się na nową wartość obliczoną jako:
  new_speed = base_speed + random.uniform(-max_change, +max_change)
Wynik jest zawsze ograniczany (clamped) do zakresu 
[min_speed_kmh, max_speed_kmh].
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
import random
import time
import math

class RandomSpeedGeneratorNode(Node):
    def __init__(self):
        super().__init__('random_speed_generator')
        
        # --- DEKLARACJA PARAMETRÓW ---
        self.declare_parameter('publish_freq_hz', 20.0)
        
        # <--- ZMIANA: Nowe parametry prędkości ---
        self.declare_parameter('min_speed_kmh', 2.8)    # Minimalna prędkość wyjściowa (globalny limit)
        self.declare_parameter('max_speed_kmh', 6.8)    # Maksymalna prędkość wyjściowa (globalny limit)
        self.declare_parameter('base_speed_kmh', 4.8)   # Prędkość bazowa, wokół której oscylujemy
        self.declare_parameter('max_change_kmh', 2.0)   # Maksymalna losowa zmiana +/- od bazy
        
        # Parametry interwału (bez zmian)
        self.declare_parameter('interval_dt', 1.0)
        self.declare_parameter('min_interval_sec', 4.0)
        self.declare_parameter('max_interval_sec', 15.0)
        
        # Callback do obsługi zmian parametrów
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # --- POBRANIE I PRZELICZENIE PARAMETRÓW ---
        self.params = {}
        self.update_parameters() # Wczytuje i przelicza wszystkie parametry
        
        # --- PUBLISHER I TIMER ---
        self.publisher_ = self.create_publisher(Float64, '/target_speed', 10)
        
        self.publish_timer = self.create_timer(
            1.0 / self.params['publish_freq_hz'], 
            self.publish_signal_callback
        )
        
        # --- STAN WĘZŁA ---
        # <--- ZMIANA: Inicjalizujemy od prędkości bazowej ---
        self.current_speed_mps = self.params['base_speed_mps']
        
        # Ustaw czas pierwszego przełączenia
        self.time_of_next_switch = time.time() + self.get_random_interval()
        
        # --- LOGOWANIE ---
        self.get_logger().info('=== GENERATOR (Baza +/- Zmiana) URUCHOMIONY ===')
        self.log_current_settings()
        self.get_logger().info('Aby zakończyć, naciśnij Ctrl+C.')

    def update_parameters(self):
        """Wczytuje wszystkie parametry ROS i przelicza jednostki."""
        self.params['publish_freq_hz'] = self.get_parameter('publish_freq_hz').get_parameter_value().double_value
        
        # <--- ZMIANA: Wczytanie nowych parametrów prędkości ---
        self.params['min_speed_kmh'] = self.get_parameter('min_speed_kmh').get_parameter_value().double_value
        self.params['max_speed_kmh'] = self.get_parameter('max_speed_kmh').get_parameter_value().double_value
        self.params['base_speed_kmh'] = self.get_parameter('base_speed_kmh').get_parameter_value().double_value
        self.params['max_change_kmh'] = self.get_parameter('max_change_kmh').get_parameter_value().double_value
        
        # Wczytanie parametrów interwału (bez zmian)
        self.params['interval_dt'] = self.get_parameter('interval_dt').get_parameter_value().double_value
        self.params['min_interval_sec'] = self.get_parameter('min_interval_sec').get_parameter_value().double_value
        self.params['max_interval_sec'] = self.get_parameter('max_interval_sec').get_parameter_value().double_value

        # --- Walidacja logiki ---
        if self.params['min_interval_sec'] > self.params['max_interval_sec']:
            self.get_logger().warn('min_interval_sec > max_interval_sec. Ustawiam min = max.')
            self.params['min_interval_sec'] = self.params['max_interval_sec']
            
        # <--- ZMIANA: Walidacja prędkości ---
        if self.params['min_speed_kmh'] > self.params['max_speed_kmh']:
            self.get_logger().warn('min_speed_kmh > max_speed_kmh. Ustawiam min = max.')
            self.params['min_speed_kmh'] = self.params['max_speed_kmh']

        if self.params['max_change_kmh'] < 0.0:
            self.get_logger().warn('max_change_kmh < 0. Ustawiam na wartość absolutną.')
            self.params['max_change_kmh'] = abs(self.params['max_change_kmh'])

        # Walidacja czy base_speed mieści się w zakresie min/max
        if not (self.params['min_speed_kmh'] <= self.params['base_speed_kmh'] <= self.params['max_speed_kmh']):
            old_base = self.params['base_speed_kmh']
            self.params['base_speed_kmh'] = max(
                self.params['min_speed_kmh'], 
                min(self.params['max_speed_kmh'], self.params['base_speed_kmh'])
            )
            self.get_logger().warn(
                f"base_speed_kmh ({old_base:.1f}) jest poza zakresem "
                f"[{self.params['min_speed_kmh']:.1f}, {self.params['max_speed_kmh']:.1f}]. "
                f"Ograniczam do: {self.params['base_speed_kmh']:.1f} km/h."
            )
            
        if self.params['interval_dt'] <= 0.001:
            self.get_logger().warn('interval_dt jest zbyt małe. Ustawiam na 0.1s.')
            self.params['interval_dt'] = 0.1

        # <--- ZMIANA: Przeliczenia na m/s ---
        self.params['min_speed_mps'] = self.params['min_speed_kmh'] / 3.6
        self.params['max_speed_mps'] = self.params['max_speed_kmh'] / 3.6
        self.params['base_speed_mps'] = self.params['base_speed_kmh'] / 3.6
        self.params['max_change_mps'] = self.params['max_change_kmh'] / 3.6

    def log_current_settings(self):
        """Loguje aktualne ustawienia generatora."""
        self.get_logger().info('------------------------------------------')
        # <--- ZMIANA: Nowy log dla prędkości ---
        self.get_logger().info(f"Globalny zakres wyjściowy: {self.params['min_speed_kmh']:.1f} km/h - {self.params['max_speed_kmh']:.1f} km/h")
        self.get_logger().info(f"Prędkość bazowa: {self.params['base_speed_kmh']:.1f} km/h")
        self.get_logger().info(f"Maksymalna zmiana +/-: {self.params['max_change_kmh']:.1f} km/h")
        self.get_logger().info(f" └ (w m/s: Baza {self.params['base_speed_mps']:.2f} +/- {self.params['max_change_mps']:.2f})")
        self.get_logger().info(f" └ (w m/s: Zakres {self.params['min_speed_mps']:.2f} - {self.params['max_speed_mps']:.2f})")
        
        # Logi interwału (bez zmian)
        self.get_logger().info(f"Krok interwału (interval_dt): {self.params['interval_dt']:.2f}s")
        self.get_logger().info(f"Zakres czasu trwania: {self.params['min_interval_sec']:.1f}s - {self.params['max_interval_sec']:.1f}s (jako wielokrotność dt)")
        self.get_logger().info(f"Częstotliwość publikacji: {self.params['publish_freq_hz']} Hz")
        self.get_logger().info('------------------------------------------')

    def parameter_callback(self, params):
        """Callback obsługujący dynamiczne zmiany parametrów."""
        # Zanim zaktualizujemy, sprawdźmy czy timer nie musi być odtworzony
        freq_changed = False
        for param in params:
            if param.name == 'publish_freq_hz':
                if param.value != self.params['publish_freq_hz']:
                    freq_changed = True
        
        self.update_parameters()
        
        # Jeśli zmieniła się częstotliwość, musimy odtworzyć timer
        if freq_changed:
            self.get_logger().info('Wykryto zmianę publish_freq_hz. Odtwarzam timer...')
            if self.publish_timer:
                self.publish_timer.cancel()
            self.publish_timer = self.create_timer(
                1.0 / self.params['publish_freq_hz'], 
                self.publish_signal_callback
            )
            
        self.get_logger().info('Parametry zaktualizowane:')
        self.log_current_settings()
        return SetParametersResult(successful=True)
    
    def get_random_interval(self):
        """
        Zwraca losowy czas trwania, będący wielokrotnością interval_dt
        i mieszczący się w zakresie [min_interval_sec, max_interval_sec].
        (Bez zmian)
        """
        dt = self.params['interval_dt']
        min_sec = self.params['min_interval_sec']
        max_sec = self.params['max_interval_sec']
        
        # Zapobiegaj dzieleniu przez zero lub ujemnej wartości
        if dt <= 0.001: 
            dt = 0.001

        min_multiplier = int(math.ceil(min_sec / dt))
        max_multiplier = int(math.floor(max_sec / dt))

        if min_multiplier > max_multiplier:
            self.get_logger().warn(
                f'Nie można znaleźć wielokrotności {dt}s w zakresie [{min_sec}s, {max_sec}s]. '
                f'Używam najbliższej wielokrotności minimum ({min_multiplier * dt}s).'
            )
            return min_multiplier * dt

        random_multiplier = random.randint(min_multiplier, max_multiplier)
        duration = random_multiplier * dt
        return duration

    # <--- ZMIANA: Nowa funkcja do obliczania prędkości ---
    def calculate_next_speed_mps(self):
        """
        Oblicza nową prędkość jako Baza +/- Losowa Zmiana,
        z wynikiem ograniczonym do [min_speed, max_speed].
        """
        base_speed = self.params['base_speed_mps']
        max_change = self.params['max_change_mps']
        min_speed = self.params['min_speed_mps']
        max_speed = self.params['max_speed_mps']
        
        # Losuj zmianę w zakresie [-max_change, +max_change]
        random_change = random.uniform(min_speed-base_speed, max_speed-base_speed)
        
        # Oblicz nową prędkość
        new_speed = base_speed + random_change
        
        # Ogranicz (clamp) prędkość do zadanego zakresu globalnego
        clamped_speed = max(min_speed, min(max_speed, new_speed))
        clamped_speed = round(clamped_speed,1)
        
        return clamped_speed

    def publish_signal_callback(self):
        """
        Główna pętla wywoływana z dużą częstotliwością.
        Publikuje sygnał i sprawdza, czy nadszedł czas na jego zmianę.
        """
        current_time = time.time()
        
        # --- LOGIKA ZMIANY SYGNAŁU ---
        if current_time >= self.time_of_next_switch:
            
            # <--- ZMIANA: Logika zmiany sygnału ---
            
            # 1. Oblicz nową prędkość wg nowej logiki
            self.current_speed_mps = self.calculate_next_speed_mps()
            
            # 2. Ustaw czas następnego przełączenia
            self.time_of_next_switch = current_time + self.get_random_interval()
            
            # 3. Loguj zmianę
            self.get_logger().info(
                f'Zmiana: Nowa prędkość (Baza +/- Zmiana): {self.current_speed_mps*3.6:.1f} km/h'
            )
        
        msg = Float64()
        msg.data = self.current_speed_mps
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomSpeedGeneratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Zatrzymywanie generatora sygnału losowego...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()