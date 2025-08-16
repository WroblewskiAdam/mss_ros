import rclpy
from rclpy.node import Node
# Importujemy naszą nową, niestandardową wiadomość
from my_robot_interfaces.msg import Gear 
from std_msgs.msg import String
import RPi.GPIO as GPIO
import json
import psutil
import time

# --- Konfiguracja pinów GPIO ---
PIN_CLUTCH = 26   # Sprzęgło
PIN_GEAR_1 = 19  # Bieg 1
PIN_GEAR_2 = 13  # Bieg 2
PIN_GEAR_3 = 5  # Bieg 3
PIN_GEAR_4 = 6  # Bieg 4

class GearReaderNode(Node):
    """
    Węzeł ROS2 do odczytu stanu sprzęgła i biegów z pinów GPIO.
    Publikuje dane na topiku "/gears" używając niestandardowej wiadomości Gear.msg.
    Brak aktywnego sygnału z pinów biegów jest interpretowany jako bieg neutralny (0).
    """
    def __init__(self):
        super().__init__('gear_reader_node')
        self.get_logger().info("Uruchamianie węzła odczytu biegów (z wiadomością Gear.msg)...")

        self.gear_pins = {
            1: PIN_GEAR_1,
            2: PIN_GEAR_2,
            3: PIN_GEAR_3,
            4: PIN_GEAR_4,
        }
        
        self.setup_gpio()

        # --- Publisher używający nowej wiadomości 'Gear' ---
        self.publisher_ = self.create_publisher(Gear, 'gears', 10)
        
        # === NOWY PUBLISHER: Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/gear_reader_node', 10)
        # === NOWY TIMER: Health reporting co 5 sekund ===
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Węzeł gotowy. Publikuje na topiku /gears.")

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status GPIO
            gpio_status = "OK"
            try:
                # Sprawdź czy GPIO jest dostępne
                GPIO.input(PIN_CLUTCH)
                gpio_status = "OK"
            except Exception:
                gpio_status = "ERROR"
            
            # Sprawdź status wątku timer
            timer_status = "OK" if hasattr(self, 'timer') else "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if gpio_status == "ERROR":
                errors.append("Problem z GPIO")
            if timer_status == "ERROR":
                errors.append("Timer nieaktywny")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'gpio_status': gpio_status,
                'timer_status': timer_status,
                'clutch_pin': PIN_CLUTCH,
                'gear_pins': list(self.gear_pins.values()),
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

    def setup_gpio(self):
        """Konfiguruje piny GPIO jako wejścia z rezystorem pull-down."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PIN_CLUTCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        for pin in self.gear_pins.values():
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.get_logger().info("Skonfigurowano piny GPIO.")

    def read_gear_state(self):
        """
        Odczytuje, który bieg jest aktywny.
        Zwraca 0 (bieg neutralny), jeśli żaden pin nie jest w stanie wysokim.
        """
        for gear, pin in self.gear_pins.items():
            if GPIO.input(pin) == GPIO.HIGH:
                return gear  # Zwraca numer biegu (1-4)
        return 0  # Zwraca 0, jeśli żaden bieg nie jest aktywny

    def read_clutch_state(self):
        """Odczytuje stan sprzęgła."""
        return 1 if GPIO.input(PIN_CLUTCH) == GPIO.HIGH else 0

    def timer_callback(self):
        """Główna pętla wywoływana przez timer."""
        # Stworzenie instancji naszej nowej wiadomości
        msg = Gear()
        
        # === ZMIANA: Dodajemy znacznik czasu ===
        msg.header.stamp = self.get_clock().now().to_msg()
        # ======================================
        
        # Wypełnienie pól wiadomości
        msg.gear = self.read_gear_state()
        msg.clutch_state = self.read_clutch_state()
        
        self.publisher_.publish(msg)

    def destroy_node(self):
        """Sprzątanie GPIO przy zamykaniu węzła."""
        self.get_logger().info("Czyszczenie pinów GPIO.")
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gear_reader_node = GearReaderNode()
    try:
        rclpy.spin(gear_reader_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            gear_reader_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()