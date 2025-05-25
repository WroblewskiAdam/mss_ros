# servo_controller/servo_controller/servo_teleop_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading

class ServoTeleop(Node):
    def __init__(self):
        super().__init__('servo_teleop')

        # Publisher, który wysyła docelowy kąt do węzła serwa.
        # Upewniamy się, że temat jest zgodny z subskrypcją w 'servo_node.py'.
        self.publisher_ = self.create_publisher(Int32, 'servo/set_angle', 10)

        # Timer do cyklicznego publikowania z częstotliwością 20 Hz
        timer_period = 1.0 / 20.0  # 50 milisekund
        self.timer = self.create_timer(timer_period, self.publish_callback)

        # Zmienna do przechowywania docelowego kąta i blokada do synchronizacji wątków
        self.target_angle = 0
        self.lock = threading.Lock()

        self.get_logger().info('Węzeł teleoperacji serwa uruchomiony.')
        self.get_logger().info('Wpisz kąt (0-180) i naciśnij Enter, aby zaktualizować pozycję.')
        self.get_logger().info('Wpisz "q", aby wyjść.')

        # Uruchomienie osobnego wątku do obsługi wejścia z klawiatury
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def publish_callback(self):
        """
        Funkcja wywoływana przez timer. Publikuje aktualnie ustawiony kąt docelowy.
        """
        msg = Int32()
        with self.lock:
            msg.data = self.target_angle
        self.publisher_.publish(msg)

    def input_loop(self):
        """
        Pętla w osobnym wątku, która czeka na dane od użytkownika i aktualizuje kąt docelowy.
        """
        while rclpy.ok():
            try:
                user_input = input('Podaj nowy kąt (0-180): ')
                if user_input.lower() == 'q':
                    # Wywołanie rclpy.shutdown() z innego wątku, aby zakończyć program
                    self.get_logger().info('Zamykanie...')
                    rclpy.shutdown()
                    break

                angle = int(user_input)

                if 0 <= angle <= 180:
                    with self.lock:
                        self.target_angle = angle
                    self.get_logger().info(f'Ustawiono nowy kąt docelowy: {self.target_angle}')
                else:
                    self.get_logger().warn('Kąt poza zakresem (0-180). Nie zmieniono wartości.')

            except ValueError:
                self.get_logger().warn('Nieprawidłowe dane wejściowe. Proszę podać liczbę całkowitą.')
            except EOFError:
                # Wciśnięcie Ctrl+D kończy pętlę
                break

def main(args=None):
    rclpy.init(args=args)
    servo_teleop_node = ServoTeleop()
    try:
        rclpy.spin(servo_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        servo_teleop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()