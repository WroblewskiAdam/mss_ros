# speed_controller/speed_controller/speed_teleop_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 # Wiadomość dla prędkości
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SpeedTeleop(Node):
    def __init__(self):
        super().__init__('speed_teleop_node')

        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Pobranie nazwy topiku z parametru (lub wartość domyślna)
        self.declare_parameter('target_speed_topic', '/target_speed')
        target_speed_topic_name = self.get_parameter('target_speed_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(
            Float64,
            target_speed_topic_name, # Używamy nazwy z parametru
            default_qos
        )
        
        self.target_speed_mps = 1.3359 # Domyślnie np. prędkość jałowa
        self.lock = threading.Lock()

        self.get_logger().info('Węzeł teleoperacji prędkości uruchomiony.')
        self.get_logger().info(f"Publikuje na {target_speed_topic_name}")
        self.get_logger().info("Aby ZAMKNĄĆ, wpisz 'q' i Enter.")
        self.get_logger().info("UWAGA: Ctrl+C to AWARYJNE wyjście.")

        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

        # Timer do okresowej publikacji, aby zapewnić, że ostatnia wartość jest wysyłana
        self.publish_timer = self.create_timer(0.5, self.publish_speed)


    def publish_speed(self):
        msg = Float64()
        with self.lock:
            msg.data = self.target_speed_mps
        self.publisher_.publish(msg)

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input('Podaj nową prędkość zadaną [m/s] lub "q", aby wyjść: ')

                if user_input.lower() == 'q':
                    self.get_logger().info("Inicjowanie zamknięcia teleoperacji prędkości...")
                    rclpy.shutdown()
                    break 

                speed = float(user_input)
                if speed >= 0: # Prędkość nie powinna być ujemna
                    with self.lock:
                        self.target_speed_mps = speed
                    self.get_logger().info(f'Ustawiono nową prędkość zadaną: {self.target_speed_mps:.2f} m/s')
                    self.publish_speed() # Opublikuj od razu po zmianie
                else:
                    self.get_logger().warn('Prędkość nie może być ujemna.')
            
            except (ValueError, EOFError):
                if not rclpy.ok():
                    break 
                self.get_logger().warn('Nieprawidłowe dane wejściowe.')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeedTeleop()
        rclpy.spin(node) # Potrzebne do działania timera
    except KeyboardInterrupt:
        print("\n[AWARYJNE WYJŚCIE] Naciśnięto Ctrl+C. Program został przerwany.")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok(): # Sprawdź ponownie, bo shutdown mógł być wywołany z wątku
            rclpy.shutdown()
        print("Węzeł teleoperacji prędkości zakończył działanie.")

if __name__ == '__main__':
    main()