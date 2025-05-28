# servo_controller/servo_controller/servo_node.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import StampedInt32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from adafruit_servokit import ServoKit
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Parametr dla watchdoga
        self.declare_parameter('watchdog_timeout_sec', 0.2)
        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').get_parameter_value().double_value

        # Inicjalizacja sprzętu
        try:
            self.kit = ServoKit(channels=16)
        except Exception as e:
            self.get_logger().error(f"Nie udało się zainicjować ServoKit (PCA9685): {e}")
            rclpy.shutdown()
            return
        self.servo_channel = 0
        self.kit.servo[self.servo_channel].set_pulse_width_range(500, 2500)
        
        # QoS i subskrypcja
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

        # Publisher i timery
        self.position_publisher = self.create_publisher(StampedInt32, 'servo/position', qos_profile=sensor_qos_profile)
        self.SERVO_SPEED_DEGPS = 750.0 # Prędkość serwa w stopniach/sekundę
        movement_timer_period = 1.0 / 100.0 # 100 Hz
        self.movement_timer = self.create_timer(movement_timer_period, self.movement_step_callback)
        publish_timer_period = 1.0 / 50.0 # 50 Hz
        self.publish_timer = self.create_timer(publish_timer_period, self.publish_position_callback)

        # Logika Watchdoga
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        watchdog_timer_period = 0.2 # Sprawdzaj 5 razy na sekundę
        self.watchdog_timer = self.create_timer(watchdog_timer_period, self.watchdog_callback)

        # Zmienne stanu serwa
        self.current_simulated_angle = 0.0
        self.target_angle = 0.0

        self.get_logger().info('Servo controller node started.')
        self.get_logger().info(f"Watchdog aktywny. Timeout: {self.watchdog_timeout}s.")
        self.set_initial_angle(0)

    def set_target_angle_callback(self, msg):
        # Resetujemy czas watchdoga przy każdej nowej komendzie
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        
        target_angle = msg.data
        if 0 <= target_angle <= 180:
            self.target_angle = float(target_angle)
        else:
            self.get_logger().warn(f"Received angle {target_angle} out of bounds (0-180). Ignoring command.")

    def watchdog_callback(self):
        """Sprawdza, czy nie upłynął czas od ostatniej wiadomości."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_last_msg = current_time - self.last_msg_time

        if time_since_last_msg > self.watchdog_timeout:
            if self.target_angle != 0.0:
                self.get_logger().warn(f"Watchdog timeout! Brak komendy przez >{self.watchdog_timeout:.1f}s. Ustawiam kąt na 0.")
                self.target_angle = 0.0
                # Resetujemy czas, aby komunikat nie pojawiał się bez przerwy
                self.last_msg_time = current_time

    # === PRZYWRÓCONY KOD ===
    def set_initial_angle(self, angle: int):
        """Ustawia pozycję początkową serwa, omijając symulację ruchu."""
        safe_angle = float(max(0, min(180, angle)))
        self.current_simulated_angle = safe_angle
        self.target_angle = safe_angle
        self.kit.servo[self.servo_channel].angle = int(safe_angle)
        
    def movement_step_callback(self):
        """Symuluje i wykonuje płynny ruch serwa do pozycji docelowej."""
        error = self.target_angle - self.current_simulated_angle
        # Jeśli jesteśmy wystarczająco blisko celu, nic nie rób
        if abs(error) < 0.01:
            return

        # Oblicz maksymalny ruch w tym kroku czasowym
        tick_duration = self.movement_timer.timer_period_ns / 1e9
        max_step = self.SERVO_SPEED_DEGPS * tick_duration
        
        # Ustaw kierunek ruchu
        step = max_step if error > 0 else -max_step
        
        # Jeśli krok jest większy niż pozostały błąd, wykonaj tylko ruch do celu
        if abs(error) < abs(step):
            step = error
            
        self.current_simulated_angle += step
        # Wyślij komendę do fizycznego serwa
        self.kit.servo[self.servo_channel].angle = int(round(self.current_simulated_angle))

    def publish_position_callback(self):
        """Publikuje aktualną (symulowaną) pozycję serwa."""
        msg = StampedInt32()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = int(round(self.current_simulated_angle))
        self.position_publisher.publish(msg)
    # =========================

    def set_angle_for_shutdown(self, angle_degrees: int):
        """Awaryjne ustawienie serwa na konkretny kąt podczas zamykania."""
        try:
            self.kit.servo[self.servo_channel].angle = angle_degrees
            time.sleep(0.5)
        except Exception as e:
            print(f"Error setting servo angle to {angle_degrees} during shutdown: {e}")

    def destroy_node(self):
        """Czyści zasoby przy zamykaniu."""
        if hasattr(self, 'kit'):
            self.get_logger().info("Wywołano destroy_node. Ustawiam serwo na 0 dla pewności.")
            self.set_angle_for_shutdown(0)
        super().destroy_node()

def main(args=None):
    # Funkcja main pozostaje bez zmian
    rclpy.init(args=args)
    servo_controller_node = None
    try:
        servo_controller_node = ServoController()
        if rclpy.ok() and hasattr(servo_controller_node, 'kit'): 
            rclpy.spin(servo_controller_node)
    except KeyboardInterrupt:
        if servo_controller_node:
            servo_controller_node.get_logger().info('Keyboard Interrupt! Shutting down...')
    finally:
        if servo_controller_node:
            servo_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()