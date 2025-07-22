import rclpy
from rclpy.node import Node
# Importujemy naszą nową, niestandardową wiadomość
from my_robot_interfaces.msg import Gear 
import RPi.GPIO as GPIO

# --- Konfiguracja pinów GPIO ---
PIN_CLUTCH = 5   # Sprzęgło
PIN_GEAR_1 = 6   # Bieg 1
PIN_GEAR_2 = 13  # Bieg 2
PIN_GEAR_3 = 19  # Bieg 3
PIN_GEAR_4 = 26  # Bieg 4

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
        
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Węzeł gotowy. Publikuje na topiku /gears.")

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
        
        # Wypełnienie pól wiadomości - kod jest teraz bardzo czytelny
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