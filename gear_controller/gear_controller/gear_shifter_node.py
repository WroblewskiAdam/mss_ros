import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # Można też stworzyć własny, ale użyjemy gotowego dla uproszczenia
import RPi.GPIO as GPIO
import time

# ros2 service call /gear_shift_down std_srvs/srv/SetBool "{data: true}"
class GearShifter(Node):
    def __init__(self):
        super().__init__('gear_shifter')

        self.relay_up_pin = 25
        self.relay_down_pin = 20

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_up_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.relay_down_pin, GPIO.OUT, initial=GPIO.LOW)

        self.srv = self.create_service(SetBool, 'gear_shift_up', self.shift_up_callback)
        self.srv2 = self.create_service(SetBool, 'gear_shift_down', self.shift_down_callback)

        self.get_logger().info('Gear shifter node ready.')

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


