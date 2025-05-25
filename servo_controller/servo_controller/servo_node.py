import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from adafruit_servokit import ServoKit
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # --- Parametry i stałe ---
        self.SERVO_SPEED_DEGPS = 750.0  # Prędkość serwa w stopniach/sekundę
        MOVEMENT_FREQUENCY = 100.0      # Częstotliwość pętli ruchu (wysoka dla płynności)
        PUBLISH_FREQUENCY = 20.0        # Docelowa częstotliwość publikowania pozycji

        # --- Zmienne stanu ---
        self.current_simulated_angle = 0.0
        self.target_angle = 0.0

        # --- Konfiguracja serwo ---
        try:
            self.kit = ServoKit(channels=16)
        except Exception as e:
            self.get_logger().error(f"Nie udało się zainicjować ServoKit (PCA9685): {e}")
            rclpy.shutdown()
            return

        self.servo_channel = 0
        self.kit.servo[self.servo_channel].set_pulse_width_range(500, 2500)
        
        # --- Subskrypcja ---
        self.subscription = self.create_subscription(
            Int32,
            'servo/set_angle',
            self.set_target_angle_callback,
            10)
        
        # --- Publisher ---
        self.position_publisher = self.create_publisher(Int32, 'servo/position', 10)
        
        # === NOWA ARCHITEKTURA: DWA TIMERY ===
        # 1. Timer do obliczania RUCHU serwa (wysoka częstotliwość)
        movement_timer_period = 1.0 / MOVEMENT_FREQUENCY
        self.movement_timer = self.create_timer(movement_timer_period, self.movement_step_callback)
        
        # 2. Timer do ciągłego PUBLIKOWANIA pozycji (zadana częstotliwość)
        publish_timer_period = 1.0 / PUBLISH_FREQUENCY
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_position_callback)
        # ====================================
        
        self.get_logger().info('Servo controller node started.')
        self.get_logger().info(f'Movement simulation at {MOVEMENT_FREQUENCY}Hz.')
        self.get_logger().info(f'Position publishing at {PUBLISH_FREQUENCY}Hz.')

        self.set_initial_angle(0)

    def set_initial_angle(self, angle: int):
        """Ustawia pozycję początkową natychmiast."""
        safe_angle = float(max(0, min(180, angle)))
        self.current_simulated_angle = safe_angle
        self.target_angle = safe_angle
        self.kit.servo[self.servo_channel].angle = int(safe_angle)
        
    def set_target_angle_callback(self, msg):
        """Callback dla /servo/set_angle. Ustawia tylko cel."""
        target_angle = msg.data
        if 0 <= target_angle <= 180:
            self.get_logger().info(f"Received command. New target: {target_angle} degrees.")
            self.target_angle = float(target_angle)
        else:
            self.get_logger().warn(f"Received angle {target_angle} out of bounds (0-180). Ignoring command.")

    def movement_step_callback(self):
        """
        Pętla RUCHU. Oblicza nowy kąt i wysyła komendę do fizycznego serwa.
        Nie publikuje żadnych wiadomości.
        """
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
        """
        Pętla PUBLIKOWANIA. Działa zawsze z zadaną częstotliwością.
        Pobiera ostatni obliczony kąt i go publikuje.
        """
        msg = Int32()
        msg.data = int(round(self.current_simulated_angle))
        self.position_publisher.publish(msg)

    def set_angle_for_shutdown(self, angle_degrees: int):
        """Ustawia kąt przy zamykaniu."""
        try:
            self.kit.servo[self.servo_channel].angle = angle_degrees
        except Exception as e:
            self.get_logger().error(f"Error setting servo angle to {angle_degrees} during shutdown: {e}")

def main(args=None):
    rclpy.init(args=args)
    servo_controller_node = None
    try:
        servo_controller_node = ServoController()
        if rclpy.ok() and hasattr(servo_controller_node, 'kit'): 
            rclpy.spin(servo_controller_node)
    except KeyboardInterrupt:
        if servo_controller_node:
            servo_controller_node.get_logger().info('Keyboard Interrupt detected! Shutting down...')
    finally:
        if servo_controller_node and rclpy.ok() and hasattr(servo_controller_node, 'kit'):
            servo_controller_node.get_logger().info('Setting servo to 0 before shutdown.')
            servo_controller_node.set_angle_for_shutdown(0)
            time.sleep(0.5)
            servo_controller_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()