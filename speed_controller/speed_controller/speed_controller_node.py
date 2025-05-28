# speed_controller/speed_controller/speed_controller_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from my_robot_interfaces.msg import StampedInt32,GpsRtk #
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time # Potrzebne do pauzy po ustawieniu serwa

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # --- Deklaracja parametrów ---
        self.declare_parameter('kp', 48.77)
        self.declare_parameter('ki', 83.49)
        self.declare_parameter('v_idle', 1.3359)
        self.declare_parameter('servo_min_angle', 0)
        self.declare_parameter('servo_max_angle', 150)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('target_speed_topic', '/target_speed')
        self.declare_parameter('current_speed_topic', '/gps_rtk_data')
        self.declare_parameter('servo_command_topic', '/servo/set_angle')
        self.declare_parameter('output_min', 0.0) 
        self.declare_parameter('output_max', 150.0)

        # --- Pobranie parametrów ---
        self.Kp = self.get_parameter('kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('ki').get_parameter_value().double_value
        self.v_idle = self.get_parameter('v_idle').get_parameter_value().double_value
        self.servo_min_angle = self.get_parameter('servo_min_angle').get_parameter_value().integer_value
        self.servo_max_angle = self.get_parameter('servo_max_angle').get_parameter_value().integer_value
        self.controller_frequency = self.get_parameter('controller_frequency').get_parameter_value().double_value
        
        self.output_min = self.get_parameter('output_min').get_parameter_value().double_value
        self.output_max = self.get_parameter('output_max').get_parameter_value().double_value

        target_speed_topic_name = self.get_parameter('target_speed_topic').get_parameter_value().string_value
        current_speed_topic_name = self.get_parameter('current_speed_topic').get_parameter_value().string_value
        servo_command_topic_name = self.get_parameter('servo_command_topic').get_parameter_value().string_value
        
        self.dt_controller = 1.0 / self.controller_frequency

        # --- Zmienne regulatora ---
        self.current_speed_mps = 0.0
        self.target_speed_mps = self.v_idle 
        self.integral_sum = 0.0
        
        # --- QoS ---
        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subskrypcje ---
        self.target_speed_sub = self.create_subscription(
            Float64,
            target_speed_topic_name,
            self.target_speed_callback,
            default_qos
        )
        self.current_speed_sub = self.create_subscription(
            GpsRtk,
            current_speed_topic_name,
            self.current_speed_callback,
            default_qos
        )

        # --- Publisher ---
        self.servo_command_pub = self.create_publisher(
            StampedInt32, 
            servo_command_topic_name,
            default_qos
        )

        # --- Timer dla pętli regulatora ---
        self.controller_timer = self.create_timer(self.dt_controller, self.controller_loop)

        self.get_logger().info(f"Węzeł regulatora prędkości PI uruchomiony.")
        # ... (reszta logów informacyjnych) ...


    def target_speed_callback(self, msg):
        if msg.data < self.v_idle:
            self.target_speed_mps = self.v_idle
            self.get_logger().warn(
                f"Zadana prędkość {msg.data:.2f} m/s jest niższa niż v_idle ({self.v_idle:.2f} m/s). "
                f"Ustawiam na v_idle."
            )
        else:
            self.target_speed_mps = msg.data

    def current_speed_callback(self, msg):
        self.current_speed_mps = msg.speed_mps

    def controller_loop(self):
        target_speed_dev = self.target_speed_mps - self.v_idle
        current_speed_dev = self.current_speed_mps - self.v_idle
        error = target_speed_dev - current_speed_dev
        output_p = self.Kp * error
        unbounded_control_signal = output_p + self.integral_sum 
        saturated_control_signal = max(self.output_min, min(self.output_max, unbounded_control_signal))
        
        output_i_potential_contribution = self.Ki * error * self.dt_controller
        
        if (unbounded_control_signal >= self.output_max and error > 0) or \
           (unbounded_control_signal <= self.output_min and error < 0):
            pass 
        else:
            self.integral_sum += output_i_potential_contribution

        servo_angle_final = saturated_control_signal 
        servo_msg = StampedInt32()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.data = int(round(servo_angle_final))
        self.servo_command_pub.publish(servo_msg)
        
        self.get_logger().info(
            f"TgtDev:{target_speed_dev:.2f} CurDev:{current_speed_dev:.2f} Err:{error:.2f} "
            f"P_out:{output_p:.2f} I_sum:{self.integral_sum:.2f} ServoCmd:{servo_angle_final:.1f}"
        )

    # === NOWA METODA do ustawiania serwa na 0 ===
    def set_servo_to_zero_and_wait(self):
        self.get_logger().info("Ustawianie serwa na 0 stopni przed zamknięciem...")
        servo_msg = StampedInt32()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.data = int(self.servo_min_angle) # Używamy zdefiniowanego minimum (zakładamy, że to 0)
        
        # Publikujemy kilka razy, aby dać szansę węzłowi serwa na odbiór
        # i serwu na fizyczne przesunięcie się.
        for _ in range(10): # Zwiększono liczbę prób dla pewności
            self.servo_command_pub.publish(servo_msg)
            time.sleep(0.05) # Krótka pauza między publikacjami
        
        self.get_logger().info(f"Komenda ustawienia serwa na {self.servo_min_angle} stopni wysłana. Oczekiwanie na wykonanie...")
        time.sleep(0.5) # Dłuższa pauza, aby serwo miało czas fizycznie wrócić do pozycji 0
                       # Czas ten może zależeć od prędkości Twojego serwa

    def destroy_node(self):
        """Metoda czyszczenia wywoływana przy zamykaniu węzła."""
        self.get_logger().info('Rozpoczynanie procedury destroy_node dla SpeedControllerNode.')
        # Tutaj nie wywołujemy set_servo_to_zero_and_wait, 
        # bo to powinno być obsłużone w sekcji finally w main()
        # po przerwaniu KeyboardInterrupt.
        super().destroy_node()
        self.get_logger().info('SpeedControllerNode.destroy_node() zakończone.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeedControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: # Sprawdź, czy node został pomyślnie utworzony
            node.get_logger().info('Przerwano przez użytkownika (Ctrl+C)! Rozpoczynam eleganckie zamykanie...')
            node.set_servo_to_zero_and_wait() # <-- Ustaw serwo na 0 przed pełnym zamknięciem
    except Exception as e:
        if node:
            node.get_logger().error(f"Wystąpił nieoczekiwany błąd: {e}", exc_info=True)
        else:
            print(f"Wystąpił krytyczny błąd przed inicjalizacją węzła SpeedControllerNode: {e}")
    finally:
        if node:
            node.get_logger().info('Finalne zamykanie węzła SpeedControllerNode...')
            node.destroy_node() # Standardowe czyszczenie rclpy
        if rclpy.ok():
            rclpy.shutdown()
        print("Węzeł regulatora prędkości zakończył działanie.")

if __name__ == '__main__':
    main()