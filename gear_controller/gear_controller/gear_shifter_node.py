import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import lgpio  # ZMIANA: Import nowej biblioteki
import time
import json
import psutil
from std_msgs.msg import String

class GearShifter(Node):
    def __init__(self):
        super().__init__('gear_shifter')

        self.relay_up_pin = 25
        self.relay_down_pin = 20

        # ZMIANA: Inicjalizacja GPIO za pomocą lgpio
        try:
            self.chip_handle = lgpio.gpiochip_open(4) # Otwórz główny chip GPIO
            # Ustaw piny jako wyjścia z początkowym stanem niskim (0)
            lgpio.gpio_claim_output(self.chip_handle, self.relay_up_pin, 0)
            lgpio.gpio_claim_output(self.chip_handle, self.relay_down_pin, 0)
        except lgpio.error as e:
            self.get_logger().error(f"Błąd inicjalizacji GPIO z lgpio: {e}")
            rclpy.shutdown()
            return

        self.srv = self.create_service(SetBool, 'gear_shift_up', self.shift_up_callback)
        self.srv2 = self.create_service(SetBool, 'gear_shift_down', self.shift_down_callback)

        self.health_pub = self.create_publisher(String, '/mss/node_health/gear_shifter', 10)
        self.health_timer = self.create_timer(5.0, self.publish_health)

        self.get_logger().info('Gear shifter node ready.')

    # Funkcja publish_health pozostaje bez zmian

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
        # ZMIANA: Sposób sterowania pinami
        lgpio.gpio_write(self.chip_handle, pin, 1)  # Ustaw stan wysoki
        time.sleep(0.3)
        lgpio.gpio_write(self.chip_handle, pin, 0)  # Ustaw stan niski

    def destroy_node(self):
        # ZMIANA: Prawidłowe zwolnienie zasobów lgpio
        if hasattr(self, 'chip_handle'):
            lgpio.gpiochip_close(self.chip_handle)
            self.get_logger().info("Zasoby lgpio zostały zwolnione.")
        super().destroy_node()

    # Funkcja publish_health i main pozostają bez zmian
    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Sprawdź status GPIO
            gpio_status = "OK"
            try:
                # Sprawdź, czy uchwyt jest nadal ważny
                lgpio.gpio_read(self.chip_handle, self.relay_up_pin)
                gpio_status = "OK"
            except Exception:
                gpio_status = "ERROR"
            
            # ... reszta funkcji bez zmian ...
            errors = []
            if gpio_status == "ERROR":
                errors.append("Problem z GPIO")
            
            health_data = {
                'status': 'running' if not errors else 'error',
                'timestamp': time.time(),
                'gpio_status': gpio_status,
                'relay_up_pin': self.relay_up_pin,
                'relay_down_pin': self.relay_down_pin,
                'errors': errors,
                'warnings': [],
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent
            }
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
        except Exception as e:
            self.get_logger().error(f"Błąd podczas publikowania health status: {e}")

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