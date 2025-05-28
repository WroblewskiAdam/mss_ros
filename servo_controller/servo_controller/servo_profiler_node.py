import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import StampedInt32
import time
import signal
import threading  # <--- FIX 1: Dodany brakujący import

# Globalna referencja do węzła, aby handler sygnału miał do niego dostęp
_profiler_node_instance = None


class ServoProfilerNode(Node):
    def __init__(self):
        super().__init__('servo_profiler_node')

        self.declare_parameter('start_angle', 0)
        self.declare_parameter('end_angle', 50)
        self.declare_parameter('angle_step', 25)
        self.declare_parameter('hold_duration_sec', 1.0)
        self.declare_parameter('initial_delay_sec', 1.0)

        self.start_angle = self.get_parameter('start_angle').get_parameter_value().integer_value
        self.end_angle = self.get_parameter('end_angle').get_parameter_value().integer_value
        self.angle_step = self.get_parameter('angle_step').get_parameter_value().integer_value
        self.hold_duration = self.get_parameter('hold_duration_sec').get_parameter_value().double_value
        self.initial_delay = self.get_parameter('initial_delay_sec').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(
            StampedInt32, 'servo/set_angle', 10
        )
        
        self.get_logger().info("Węzeł profilera serwa uruchomiony.")
        
        # Uruchamiamy główną logikę w osobnym wątku
        self.profiling_thread = threading.Thread(target=self.run_profiling_sequence, daemon=True)
        self.profiling_thread.start()

    def run_profiling_sequence(self):
        """Główna pętla profilowania działająca w osobnym wątku."""
        self.get_logger().info(f"Profilowanie od {self.start_angle} do {self.end_angle} stopni...")
        time.sleep(self.initial_delay)
        
        current_angle = self.start_angle
        while current_angle <= self.end_angle and rclpy.ok():
            self.get_logger().info(f"Ustawianie kąta serwa na: {current_angle} stopni.")
            msg = StampedInt32()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data = int(current_angle)
            self.publisher_.publish(msg)
            
            time.sleep(self.hold_duration)
            current_angle += self.angle_step

        if rclpy.ok():
            self.get_logger().info("Sekwencja profilowania zakończona. Inicjowanie zamknięcia.")
            self.initiate_shutdown()
        
    def initiate_shutdown(self):
        """Eleganckie zamknięcie - ustawia serwo na 0 i zamyka rclpy."""
        self.get_logger().info("Ustawianie serwa na 0 przed wyjściem...")
        msg = StampedInt32()
        msg.data = 0
        
        for _ in range(5):
            if not rclpy.ok(): break
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            time.sleep(0.02) 

        self.get_logger().info("Komenda 'kąt=0' wysłana. Zamykanie rclpy.")
        if rclpy.ok():
            rclpy.shutdown()


def signal_handler(sig, frame):
    """Globalna funkcja, która przechwytuje Ctrl+C i woła eleganckie zamknięcie."""
    print("\nOtrzymano Ctrl+C! Inicjuję eleganckie zamykanie...")
    if _profiler_node_instance:
        # Wywołujemy naszą bezpieczną procedurę zamykania
        _profiler_node_instance.initiate_shutdown()
    else:
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)
    
    global _profiler_node_instance
    node = None
    
    try:
        node = ServoProfilerNode()
        _profiler_node_instance = node
        rclpy.spin(node)
    except KeyboardInterrupt: # <--- FIX 2: Uproszczona i poprawna obsługa wyjątków
        if node:
             node.get_logger().info("Spin przerwany przez Ctrl+C, program zakończy działanie.")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Servo Profiler zakończył działanie.")

if __name__ == '__main__':
    main()