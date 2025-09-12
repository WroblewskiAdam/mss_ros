import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from my_robot_interfaces.msg import Gear, StampedInt32
from std_msgs.msg import Float64, String
from my_robot_interfaces.msg import GpsRtk  # Import dla prędkości aktualnej
import json
import psutil
import time

class GearManagerNode(Node):
    """
    Węzeł do automatycznego zarządzania zmianą półbiegów (powershift).
    Decyzje o zmianie podejmowane są na podstawie PRĘDKOŚCI AKTUALNEJ z GPS
    i znanych charakterystyk prędkości dla każdego półbiegu.
    
    Progi prędkości definiowane są w km/h (jak na liczniku ciągnika):
    - Bieg 1: do 10.4 km/h
    - Bieg 2: do 13.0 km/h  
    - Bieg 3: do 14.8 km/h
    - Bieg 4: do 20.2 km/h
    
    Logika:
    - UPSHIFT: gdy aktualna prędkość > 95% max prędkości biegu
    - DOWNSHIFT: gdy aktualna prędkość < 85% max prędkości niższego biegu
    
    Automatycznie konwertuje km/h na m/s dla obliczeń wewnętrznych.
    """
    def __init__(self):
        super().__init__('gear_manager_node')

        # --- Parametry decyzyjne oparte na charakterystyce prędkości ---
        # Progi prędkości w km/h (jak na liczniku ciągnika)
        self.declare_parameter('powershift_max_speeds_kmh', [10.4, 13.0, 14.8, 20.2])  # km/h
        self.declare_parameter('upshift_threshold_percent', 0.95)
        self.declare_parameter('downshift_threshold_percent', 0.85)
        self.declare_parameter('shift_cooldown_sec', 4.0)
        self.declare_parameter('initial_powershift', 1)
        self.declare_parameter('max_powershift', 4)

        # Konwersja km/h na m/s dla obliczeń wewnętrznych
        powershift_max_speeds_kmh = self.get_parameter('powershift_max_speeds_kmh').get_parameter_value().double_array_value
        self.powershift_max_speeds = [speed_kmh / 3.6 for speed_kmh in powershift_max_speeds_kmh]  # km/h -> m/s
        self.upshift_thresh_percent = self.get_parameter('upshift_threshold_percent').get_parameter_value().double_value
        self.downshift_thresh_percent = self.get_parameter('downshift_threshold_percent').get_parameter_value().double_value
        self.shift_cooldown = self.get_parameter('shift_cooldown_sec').get_parameter_value().double_value
        self.initial_powershift = self.get_parameter('initial_powershift').get_parameter_value().integer_value
        self.max_powershift = self.get_parameter('max_powershift').get_parameter_value().integer_value

        # --- Zmienne stanu ---
        self.current_speed = 0.0  # Prędkość aktualna z GPS (m/s)
        self.current_mechanical_gear = 0
        self.clutch_pressed = True
        self.current_powershift = 0  # ZMIANA: 0 = nieznany, będzie odczytane z /gears
        self.last_shift_time = self.get_clock().now()
        self.is_enabled = True  # NOWY: Stan włączania/wyłączania gear managera
        self.powershift_initialized = False  # NOWY: Flaga czy półbieg został odczytany

        # --- Subskrypcje ---
        self.create_subscription(Gear, '/gears', self.gear_callback, 10)
        self.create_subscription(GpsRtk, '/gps_rtk_data_filtered', self.current_speed_callback, 10)  # Prędkość aktualna z GPS

        # --- Klienci usług do zmiany biegów ---
        self.shift_up_client = self.create_client(SetBool, 'gear_shift_up')
        self.shift_down_client = self.create_client(SetBool, 'gear_shift_down')
        while not self.shift_up_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Oczekiwanie na serwis /gear_shift_up...')
        while not self.shift_down_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Oczekiwanie na serwis /gear_shift_down...')

        # --- NOWY: Serwis do włączania/wyłączania gear managera ---
        self.set_enabled_service = self.create_service(SetBool, 'gear_manager/set_enabled', self.set_enabled_callback)

        # --- Główna pętla decyzyjna ---
        self.decision_timer = self.create_timer(0.5, self.decision_loop) # 2 Hz

        # === NOWY PUBLISHER: Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/gear_manager_node', 10)
        # === NOWY TIMER: Health reporting co 5 sekund ===
        self.health_timer = self.create_timer(5.0, self.publish_health)

        self.get_logger().info("Menedżer półbiegów uruchomiony (logika oparta na profilu prędkości).")
        self.get_logger().info(f"Maksymalne prędkości półbiegów: {powershift_max_speeds_kmh} km/h")
        self.get_logger().info(f"Konwersja na m/s: {[f'{speed:.2f}' for speed in self.powershift_max_speeds]} m/s")
        self.get_logger().info("Oczekiwanie na odczyt aktualnego półbiegu z topiku /gears...")

    def gear_callback(self, msg):
        """Odbiera stan biegu (półbiegu) i sprzęgła."""
        # ZMIANA: msg.gear to jest półbieg (powershift), nie bieg główny
        if not self.powershift_initialized and msg.gear > 0:
            # Pierwszy odczyt - zainicjalizuj półbieg
            self.current_powershift = msg.gear
            self.powershift_initialized = True
            self.get_logger().info(f"Zainicjalizowano półbieg na podstawie /gears: {self.current_powershift}")
        elif self.current_powershift != msg.gear and msg.gear > 0:
            # Zmiana półbiegu (ręczna przez operatora)
            old_powershift = self.current_powershift
            self.current_powershift = msg.gear
            self.get_logger().info(f"Wykryto ręczną zmianę półbiegu: {old_powershift} -> {self.current_powershift}")
        
        # Aktualizuj stan sprzęgła
        self.clutch_pressed = (msg.clutch_state == 1)
        
    def current_speed_callback(self, msg):
        """Odbiera aktualną prędkość z GPS (filtrowaną)."""
        self.current_speed = msg.speed_mps

    def set_enabled_callback(self, request, response):
        """Callback dla serwisu włączania/wyłączania gear managera."""
        self.is_enabled = request.data
        
        if self.is_enabled:
            self.get_logger().info("Gear Manager WŁĄCZONY przez web interface")
            response.success = True
            response.message = "Gear Manager włączony"
        else:
            self.get_logger().info("Gear Manager WYŁĄCZONY przez web interface")
            response.success = True
            response.message = "Gear Manager wyłączony"
        
        return response

    async def call_shift_service(self, client, direction):
        """Asynchronicznie wywołuje usługę zmiany biegu."""
        if not client.service_is_ready():
            self.get_logger().error(f"Serwis zmiany biegów ({direction}) nie jest dostępny.")
            return False
        
        request = SetBool.Request()
        request.data = True
        future = await client.call_async(request)
        
        if future and future.success:
            self.get_logger().info(f"Pomyślnie zmieniono półbieg: {direction}")
            self.last_shift_time = self.get_clock().now()
            return True
        else:
            self.get_logger().error(f"Nie udało się zmienić półbiegu: {direction}")
            return False

    async def decision_loop(self):
        """Główna pętla, która podejmuje decyzje o zmianie półbiegów."""
        # --- NOWY: Sprawdź czy gear manager jest włączony ---
        if not self.is_enabled:
            return
        
        # --- NOWY: Sprawdź czy półbieg został zainicjalizowany ---
        if not self.powershift_initialized:
            return
        
        # --- Warunki bezpieczeństwa ---
        if self.clutch_pressed or self.current_powershift == 0 or self.current_speed == 0.0:
            return

        # --- Cooldown po ostatniej zmianie ---
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
                self.get_logger().info(f"UPSHIFT: Aktualna ({current_kmh:.1f} km/h) > Próg ({trigger_kmh:.1f} km/h) dla biegu {self.current_powershift}.")
                if await self.call_shift_service(self.shift_up_client, "GÓRA"):
                    self.current_powershift += 1
                return

        # --- LOGIKA ZMIANY BIEGU W DÓŁ ---
        if self.current_powershift > 1:
            lower_gear_index = current_gear_index - 1
            lower_gear_max_speed = self.powershift_max_speeds[lower_gear_index]
            downshift_speed_trigger = lower_gear_max_speed * self.downshift_thresh_percent

            if self.current_speed < downshift_speed_trigger:
                current_kmh = self.current_speed * 3.6
                trigger_kmh = downshift_speed_trigger * 3.6
                self.get_logger().info(f"DOWNSHIFT: Aktualna ({current_kmh:.1f} km/h) < Próg ({trigger_kmh:.1f} km/h) dla biegu {self.current_powershift - 1}.")
                if await self.call_shift_service(self.shift_down_client, "DÓŁ"):
                    self.current_powershift -= 1
                return

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status klientów usług
            shift_up_status = "OK" if self.shift_up_client.service_is_ready() else "ERROR"
            shift_down_status = "OK" if self.shift_down_client.service_is_ready() else "ERROR"
            
            # Sprawdź status timerów
            timer_status = "OK" if hasattr(self, 'decision_timer') else "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if shift_up_status == "ERROR":
                errors.append("Serwis shift_up niedostępny")
            if shift_down_status == "ERROR":
                errors.append("Serwis shift_down niedostępny")
            if timer_status == "ERROR":
                errors.append("Timer decyzyjny nieaktywny")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'shift_up_status': shift_up_status,
                'shift_down_status': shift_down_status,
                'timer_status': timer_status,
                'current_powershift': self.current_powershift,
                'current_mechanical_gear': self.current_mechanical_gear,
                'clutch_pressed': self.clutch_pressed,
                'current_speed_mps': self.current_speed,
                'current_speed_kmh': self.current_speed * 3.6,
                'is_enabled': self.is_enabled,
                'powershift_initialized': self.powershift_initialized,
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
