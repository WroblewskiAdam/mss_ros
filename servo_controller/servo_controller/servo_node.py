import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import StampedInt32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from adafruit_servokit import ServoKit
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        # ... (reszta __init__ bez zmian)
        self.SERVO_SPEED_DEGPS = 750.0
        MOVEMENT_FREQUENCY = 100.0
        PUBLISH_FREQUENCY = 50.0
        self.current_simulated_angle = 0.0
        self.target_angle = 0.0
        try:
            self.kit = ServoKit(channels=16)
        except Exception as e:
            self.get_logger().error(f"Nie udało się zainicjować ServoKit (PCA9685): {e}")
            rclpy.shutdown()
            return
        self.servo_channel = 0
        self.kit.servo[self.servo_channel].set_pulse_width_range(500, 2500)
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            StampedInt32,
            'servo/set_angle',
            self.set_target_angle_callback,
            qos_profile=sensor_qos_profile
        )
        self.position_publisher = self.create_publisher(
            StampedInt32, 
            'servo/position', 
            qos_profile=sensor_qos_profile
        )
        movement_timer_period = 1.0 / MOVEMENT_FREQUENCY
        self.movement_timer = self.create_timer(movement_timer_period, self.movement_step_callback)
        publish_timer_period = 1.0 / PUBLISH_FREQUENCY
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_position_callback)
        self.get_logger().info('Servo controller node started.')
        self.get_logger().info(f'Movement simulation at {MOVEMENT_FREQUENCY}Hz.')
        self.get_logger().info(f'Position publishing at {PUBLISH_FREQUENCY}Hz.')
        self.set_initial_angle(0)

    def set_initial_angle(self, angle: int):
        safe_angle = float(max(0, min(180, angle)))
        self.current_simulated_angle = safe_angle
        self.target_angle = safe_angle
        self.kit.servo[self.servo_channel].angle = int(safe_angle)
        
    def set_target_angle_callback(self, msg):
        target_angle = msg.data
        if 0 <= target_angle <= 180:
            self.target_angle = float(target_angle)
        else:
            self.get_logger().warn(f"Received angle {target_angle} out of bounds (0-180). Ignoring command.")

    def movement_step_callback(self):
        error = self.target_angle - self.current_simulated_angle
        if abs(error) < 0.01:
            return
        tick_duration = self.movement_timer.timer_period_ns / 1e9
        max_step = self.SERVO_SPEED_DEGPS * tick_duration
        step = max_step if error > 0 else -max_step
        if abs(error) < abs(step):
            step = error
        self.current_simulated_angle += step
        self.kit.servo[self.servo_channel].angle = int(round(self.current_simulated_angle))

    def publish_position_callback(self):
        msg = StampedInt32()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = int(round(self.current_simulated_angle))
        self.position_publisher.publish(msg)

    def set_angle_for_shutdown(self, angle_degrees: int):
        try:
            self.kit.servo[self.servo_channel].angle = angle_degrees
        except Exception as e:
            # Używamy zwykłego print(), bo logger może już nie działać
            print(f"Error setting servo angle to {angle_degrees} during shutdown: {e}")

    def destroy_node(self):
        """Metoda czyszczenia. Już nie loguje, tylko wykonuje akcje."""
        # Sprawdzamy, czy 'kit' istnieje, aby uniknąć błędu
        if hasattr(self, 'kit'):
            self.set_angle_for_shutdown(0)
            time.sleep(0.5)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    servo_controller_node = None
    try:
        servo_controller_node = ServoController()
        if rclpy.ok() and hasattr(servo_controller_node, 'kit'): 
            rclpy.spin(servo_controller_node)
    except KeyboardInterrupt:
        if servo_controller_node:
            # Logujemy zamiar zamknięcia tutaj, gdy logger na 100% działa
            servo_controller_node.get_logger().info('Keyboard Interrupt! Setting servo to 0 and shutting down...')
    finally:
        if servo_controller_node:
            # Metoda destroy_node wykona swoje zadanie (ustawienie serwa)
            servo_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()