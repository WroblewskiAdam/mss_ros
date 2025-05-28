# speed_controller/speed_controller/speed_controller_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from my_robot_interfaces.msg import StampedInt32, GpsRtk
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class SpeedControllerNode(Node):
    #
    # CAŁA ZAWARTOŚĆ KLASY SpeedControllerNode POZOSTAJE BEZ ZMIAN
    # Z WYJĄTKIEM USUNIĘCIA METODY destroy_node(), BO NIE BĘDZIE JUŻ POTRZEBNA
    #
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

    def set_servo_to_zero_and_wait(self):
        # Ta metoda jest teraz wywoływana z bloku except
        self.get_logger().info("Ustawianie serwa na 0 stopni przed zamknięciem...")
        servo_msg = StampedInt32()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.data = int(self.servo_min_angle)
        
        for _ in range(10):
            self.servo_command_pub.publish(servo_msg)
            time.sleep(0.05)
        
        self.get_logger().info(f"Komenda ustawienia serwa na {self.servo_min_angle} stopni wysłana. Oczekiwanie na wykonanie...")
        time.sleep(0.5)

#
# === NOWA WERSJA FUNKCJI main ===
#
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeedControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika (Ctrl+C)!")
        # Wykonujemy całą logikę czyszczenia natychmiast po przerwaniu,
        # zanim przejdziemy do bloku `finally`, który definitywnie zamknie ROS.
        if node:
            try:
                # Mamy bardzo małe "okno czasowe" na wysłanie tej wiadomości,
                # zanim kontekst ROS zostanie w pełni unieważniony.
                node.set_servo_to_zero_and_wait()
                print("Procedura ustawiania serwa na 0 zakończona.")
            except rclpy.errors.RCLError as e:
                # Jeśli nawet tutaj się nie uda, to znaczy, że kontekst
                # został zamknięty ekstremalnie szybko.
                print(f"Nie udało się wysłać komendy do serwa podczas zamykania: {e}")

    except Exception as e:
        if node:
            node.get_logger().error(f"Wystąpił nieoczekiwany błąd: {e}", exc_info=True)
        else:
            print(f"Wystąpił krytyczny błąd przed inicjalizacją węzła: {e}")
    finally:
        # Ten blok teraz tylko formalnie niszczy obiekt węzła
        # i zamyka rclpy, jeśli jeszcze działa.
        if node:
            # W tym momencie kontekst może być już nieważny, więc logger też może nie działać
            print('Finalne zamykanie węzła SpeedControllerNode...')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Węzeł regulatora prędkości zakończył działanie.")


if __name__ == '__main__':
    main()