# servo_controller/servo_controller/servo_teleop_node.py

import rclpy
from rclpy.node import Node
import threading
import time
from my_robot_interfaces.msg import StampedInt32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ServoTeleop(Node):
    def __init__(self):
        super().__init__('servo_teleop')

        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            StampedInt32,
            'servo/set_angle',
            qos_profile=default_qos
        )

        # === ZACHOWANY TIMER ===
        # Ta pętla jest potrzebna, aby ciągle wysyłać zadaną pozycję,
        # co utrzymuje "przy życiu" watchdog w węźle serwa.
        timer_period = 1.0 / 50.0  # 50 Hz
        self.timer = self.create_timer(timer_period, self.publish_callback)
        # =======================

        self.target_angle = 0
        self.lock = threading.Lock()

        # Uproszczone instrukcje dla użytkownika
        self.get_logger().info('Węzeł teleoperacji serwa uruchomiony.')
        self.get_logger().info("-----------------------------------------")
        self.get_logger().info("Wpisz kąt (0-180) i naciśnij Enter.")
        self.get_logger().info("Aby zakończyć, naciśnij Ctrl+C.")
        self.get_logger().info("-----------------------------------------")

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def publish_callback(self):
        """Ta funkcja jest regularnie wywoływana przez timer (50 Hz)."""
        msg = StampedInt32()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.data = self.target_angle
        self.publisher_.publish(msg)

    def input_loop(self):
        """Wątek do obsługi wejścia z terminala."""
        while rclpy.ok():
            try:
                user_input = input('Podaj nowy kąt (0-180): ')

                # === USUNIĘTA LOGIKA ZAMYKANIA ===
                # Nie ma już sprawdzania, czy user_input == 'q'.
                # Wątek po prostu obsługuje zmianę kąta.
                # =================================

                angle = int(user_input)
                if 0 <= angle <= 180:
                    with self.lock:
                        self.target_angle = angle
                    self.get_logger().info(f'Ustawiono nowy kąt docelowy: {self.target_angle}')
                else:
                    self.get_logger().warn('Kąt poza zakresem (0-180).')
            
            except (ValueError, EOFError):
                if not rclpy.ok():
                    break
                self.get_logger().warn('Nieprawidłowe dane wejściowe.')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoTeleop()
        # rclpy.spin() jest niezbędne do działania timera publikującego
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Standardowa, prosta obsługa Ctrl+C
        print("\n[INFO] Naciśnięto Ctrl+C. Zamykanie węzła.")
    finally:
        # Standardowe czyszczenie
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()