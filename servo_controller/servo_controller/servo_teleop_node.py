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

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(
            StampedInt32,
            'servo/set_angle',
            qos_profile=sensor_qos_profile
        )

        # Zapisujemy okres timera, aby móc go użyć przy zamykaniu
        self.timer_period = 1.0 / 50.0  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_callback)

        self.target_angle = 0
        self.lock = threading.Lock()

        # Zaktualizowane instrukcje dla użytkownika
        self.get_logger().info('Węzeł teleoperacji serwa uruchomiony.')
        self.get_logger().info("-----------------------------------------------------------------")
        self.get_logger().info("Aby ELEGANCKO ZAMKNĄĆ i ustawić serwo na 0, wpisz 'q' i Enter.")
        self.get_logger().warn("UWAGA: Ctrl+C to AWARYJNE wyjście i NIE ustawi serwa na 0!")
        self.get_logger().info("-----------------------------------------------------------------")


        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def publish_callback(self):
        """Ta funkcja jest regularnie wywoływana przez rclpy.spin()"""
        msg = StampedInt32()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.data = self.target_angle
        self.publisher_.publish(msg)

    def input_loop(self):
        """Wątek do obsługi wejścia, który jest jedynym sposobem na eleganckie zamknięcie."""
        while rclpy.ok():
            try:
                user_input = input('Podaj nowy kąt (0-180) lub "q", aby wyjść: ')

                # --- Logika eleganckiego zamknięcia ---
                if user_input.lower() == 'q':
                    self.get_logger().info("Inicjowanie eleganckiego zamknięcia...")
                    
                    # 1. Zmieniamy kąt docelowy na 0
                    with self.lock:
                        self.target_angle = 0
                    
                    self.get_logger().info("Ustawiono kąt docelowy na 0. Oczekiwanie na wysłanie przez timer...")
                    
                    # 2. Czekamy chwilę. W tym czasie rclpy.spin() w głównym wątku
                    #    wciąż działa i wywołuje publish_callback, który wyśle nową wartość.
                    #    Czekamy ~3 cykle timera dla pewności.
                    time.sleep(self.timer_period * 3)

                    self.get_logger().info("Komenda 'kąt=0' wysłana. Zamykanie rclpy.")
                    
                    # 3. Dopiero teraz zamykamy ROS
                    rclpy.shutdown()
                    break # Kończymy pętlę input

                # --- Normalna logika ustawiania kąta ---
                angle = int(user_input)
                if 0 <= angle <= 180:
                    with self.lock:
                        self.target_angle = angle
                    self.get_logger().info(f'Ustawiono nowy kąt docelowy: {self.target_angle}')
                else:
                    self.get_logger().warn('Kąt poza zakresem (0-180).')
            
            except (ValueError, EOFError):
                if not rclpy.ok():
                    break # Wyjdź z pętli jeśli ROS jest już zamknięty
                self.get_logger().warn('Nieprawidłowe dane wejściowe.')

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ServoTeleop()
        # spin() jest niezbędne do działania timera publikującego
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Obsługujemy Ctrl+C, ale tylko informujemy użytkownika o twardym wyjściu.
        # Nie próbujemy wykonywać żadnych akcji ROS, bo i tak się nie udadzą.
        print("\n[AWARYJNE WYJŚCIE] Naciśnięto Ctrl+C. Program został przerwany.")
    finally:
        # Standardowe czyszczenie na samym końcu
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()