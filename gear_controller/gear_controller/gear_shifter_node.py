import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # Można też stworzyć własny, ale użyjemy gotowego dla uproszczenia
import RPi.GPIO as GPIO
import time
import json
import psutil
from std_msgs.msg import String

# ros2 service call /gear_shift_down std_srvs/srv/SetBool "{data: true}"
class GearShifter(Node):
    def __init__(self):
        super().__init__('gear_shifter')

        self.relay_up_pin = 25
        self.relay_down_pin = 20

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)  # Wyłączenie ostrzeżeń o pinach już w użyciu
        GPIO.setup(self.relay_up_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.relay_down_pin, GPIO.OUT, initial=GPIO.LOW)

        self.srv = self.create_service(SetBool, 'gear_shift_up', self.shift_up_callback)
        self.srv2 = self.create_service(SetBool, 'gear_shift_down', self.shift_down_callback)

        # === NOWY PUBLISHER: Health reporting ===
        self.health_pub = self.create_publisher(String, '/mss/node_health/gear_shifter', 10)
        # === NOWY TIMER: Health reporting co 5 sekund ===
        self.health_timer = self.create_timer(5.0, self.publish_health)

        self.get_logger().info('Gear shifter node ready.')

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status GPIO
            gpio_status = "OK"
            try:
                GPIO.input(self.relay_up_pin)
                GPIO.input(self.relay_down_pin)
                gpio_status = "OK"
            except Exception:
                gpio_status = "ERROR"
            
            # Zbierz dane o błędach i ostrzeżeniach
            errors = []
            warnings = []
            
            if gpio_status == "ERROR":
                errors.append("Problem z GPIO")
            
            # Przygotuj dane health
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'gpio_status': gpio_status,
                'relay_up_pin': self.relay_up_pin,
                'relay_down_pin': self.relay_down_pin,
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

    def shift_up_callback(self, request, response):
        self.get_logger().info('Gear up requested.')
        self._trigger_relay(self.relay_up_pin)
        response.success = True
        response.message = 'Shifted up'
        return response

    def shift_down_callback(self, request, response):
        self.get_logger().info('Gear down requested.')
        self._trigger_relay(self.relay_down_pin)
        response.success = True
        response.message = 'Shifted down'
        return response

    def _trigger_relay(self, pin):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(pin, GPIO.LOW)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GearShifter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


