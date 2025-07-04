import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from my_robot_interfaces.msg import StampedInt32, GpsRtk # Upewnij się, że GpsRtk jest z my_robot_interfaces
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
# NOWY IMPORT dla obsługi zdarzeń parametrów
from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter import Parameter

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # --- Deklaracja parametrów ---
        # Dodajemy ParameterDescriptor, aby móc opisać parametry i ułatwić ich dynamiczną zmianę
        from rcl_interfaces.msg import ParameterDescriptor

        self.declare_parameter('kp', 20.00, ParameterDescriptor(description='Wzmocnienie proporcjonalne (Kp) regulatora PI'))
        self.declare_parameter('ki', 40.00, ParameterDescriptor(description='Wzmocnienie całkujące (Ki) regulatora PI'))
        self.declare_parameter('v_idle', 1.3359)
        self.declare_parameter('servo_min_angle', 0)
        self.declare_parameter('servo_max_angle', 150)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('target_speed_topic', '/target_speed')
        self.declare_parameter('current_speed_topic', '/gps_rtk_data_filtered')
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
            GpsRtk,  # Upewnij się, że ta wiadomość jest poprawnie zdefiniowana i importowana
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

        # === NOWOŚĆ: Subskrypcja do zdarzeń zmiany parametrów ===
        # W ROS 2 Humble i nowszych można użyć ParameterEventHandler,
        # ale prostszym podejściem (kompatybilnym też ze starszymi wersjami jak Foxy)
        # jest użycie wbudowanego callbacku przy deklaracji lub monitorowanie serwisu.
        # Tutaj pokażę sposób z callbackiem ustawianym przy deklaracji parametru,
        # jeśli rclpy na to pozwala w Twojej wersji, lub przez ParameterEventHandler.

        # Sposób 1: Użycie Parameter Event Handler (zalecane dla nowszych ROS 2)
        # Potrzebujesz zaimportować: from rclpy.event_handler import ParameterEventHandler
        # (Uwaga: ParameterEventHandler może nie być dostępny w starszych dystrybucjach ROS 2 w rclpy,
        #  wtedy alternatywą jest ręczne tworzenie subskrybenta do /parameter_events)

        # Alternatywnie, można użyć wbudowanego mechanizmu `add_on_set_parameters_callback`
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info(f"Węzeł regulatora prędkości PI uruchomiony.")
        self.get_logger().info(f"  Początkowe Kp: {self.Kp}, Ki: {self.Ki}")

    # === NOWOŚĆ: Callback wywoływany przy próbie zmiany parametrów ===
    def parameters_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        successful = True
        reason = ""
        for param in params:
            if param.name == "kp":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.Kp = param.value
                    self.get_logger().info(f"Zmieniono Kp na: {self.Kp}")
                else:
                    successful = False
                    reason += "Parametr 'kp' musi być typu double. "
            elif param.name == "ki":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.Ki = param.value
                    # Możesz chcieć zresetować sumę całki przy zmianie Ki
                    # self.integral_sum = 0.0
                    self.get_logger().info(f"Zmieniono Ki na: {self.Ki}")
                else:
                    successful = False
                    reason += "Parametr 'ki' musi być typu double. "
            # Możesz dodać obsługę innych parametrów, jeśli chcesz je dynamicznie zmieniać
            # np. v_idle, output_min, output_max

        return SetParametersResult(successful=successful, reason=reason)

    def target_speed_callback(self, msg):
        if msg.data < self.v_idle:
            self.target_speed_mps = self.v_idle
            # self.get_logger().warn( # Można tymczasowo wyciszyć, jeśli często się zdarza
            #     f"Zadana prędkość {msg.data:.2f} m/s jest niższa niż v_idle ({self.v_idle:.2f} m/s). "
            #     f"Ustawiam na v_idle."
            # )
        else:
            self.target_speed_mps = msg.data

    def current_speed_callback(self, msg):
        self.current_speed_mps = msg.speed_mps

    def controller_loop(self):
        target_speed_dev = self.target_speed_mps - self.v_idle
        current_speed_dev = self.current_speed_mps - self.v_idle
        error = target_speed_dev - current_speed_dev
        
        # Używamy zaktualizowanych self.Kp i self.Ki
        output_p = self.Kp * error
        
        # Potencjalny przyrost całki
        output_i_potential_contribution = self.Ki * error * self.dt_controller
        
        # Sygnał sterujący przed ograniczeniem (do sprawdzenia anty-windup)
        unbounded_control_signal = output_p + self.integral_sum + output_i_potential_contribution

        # Sprawdzenie warunków anti-windup PRZED dodaniem aktualnego przyrostu
        # Jeśli sygnał jest już na granicy I błąd powoduje dalsze narastanie w tym samym kierunku, nie całkuj
        if ( (self.integral_sum + output_p >= self.output_max and error > 0) or \
             (self.integral_sum + output_p <= self.output_min and error < 0) ) and \
           ( (unbounded_control_signal >= self.output_max) or (unbounded_control_signal <= self.output_min) ):
            # Nie dodajemy output_i_potential_contribution do self.integral_sum
            # ale output_p jest nadal uwzględniany w finalnym sygnale
            pass
        else:
            self.integral_sum += output_i_potential_contribution
            # Dodatkowe ograniczenie sumy całkującej, aby nie rosła w nieskończoność
            # To jest przydatne, jeśli output_max i output_min są daleko od zera
            # Można dostosować te granice indywidualnie dla członu całkującego
            # Np. self.integral_sum = max(self.output_min - output_p_max_possible, min(self.output_max - output_p_min_possible, self.integral_sum))
            # Dla uproszczenia, można przyjąć granice takie jak dla całego sygnału, ale to może być zbyt restrykcyjne dla I.
            # Bardziej typowe jest ograniczenie sumy całki do wartości, które pozwalają P działać w pewnym zakresie.
            # Na razie zostawimy to tak, ale warto pamiętać o możliwości "ucieczki" sumy całkującej.


        # Ostateczny sygnał sterujący po zsumowaniu P i zaktualizowanej I, przed saturacją
        final_unbounded_signal = output_p + self.integral_sum
        saturated_control_signal = max(self.output_min, min(self.output_max, final_unbounded_signal))
        
        servo_angle_final = saturated_control_signal
        servo_msg = StampedInt32()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.data = int(round(servo_angle_final))
        self.servo_command_pub.publish(servo_msg)
        
        self.get_logger().info( # Zmieniono na debug, aby nie zaśmiecać konsoli przy częstym wywoływaniu
            f"Kp:{self.Kp:.2f} Ki:{self.Ki:.2f}"
            f"TgtDev:{target_speed_dev:.2f} CurDev:{current_speed_dev:.2f} Err:{error:.2f} "
            f"P_out:{output_p:.2f} I_sum:{self.integral_sum:.2f} Unbnd:{final_unbounded_signal:.2f} ServoCmd:{servo_angle_final:.1f}"
        )

    def set_servo_to_zero_and_wait(self):
        self.get_logger().info("Ustawianie serwa na 0 stopni przed zamknięciem...")
        servo_msg = StampedInt32()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.data = int(self.servo_min_angle) # Używamy zdefiniowanego minimalnego kąta
        
        # Wyślij komendę kilkukrotnie, aby zwiększyć szansę dotarcia
        for _ in range(10): 
            self.servo_command_pub.publish(servo_msg)
            time.sleep(0.05) # Krótka pauza między wiadomościami
        
        self.get_logger().info(f"Komenda ustawienia serwa na {self.servo_min_angle} stopni wysłana. Oczekiwanie na wykonanie...")
        time.sleep(0.5) # Daj serwu czas na reakcję

# Funkcja main bez zmian
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeedControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika (Ctrl+C)!")
        if node:
            try:
                node.set_servo_to_zero_and_wait()
                print("Procedura ustawiania serwa na 0 zakończona.")
            except rclpy.errors.RCLError as e:
                print(f"Nie udało się wysłać komendy do serwa podczas zamykania: {e}")
    except Exception as e:
        if node:
            node.get_logger().error(f"Wystąpił nieoczekiwany błąd: {e}", exc_info=True)
        else:
            print(f"Wystąpił krytyczny błąd przed inicjalizacją węzła: {e}")
    finally:
        if node:
            print('Finalne zamykanie węzła SpeedControllerNode...')
            node.destroy_node() # destroy_node() jest automatycznie wywoływane przy shutdown, ale dla pewności
        if rclpy.ok():
            rclpy.shutdown()
        print("Węzeł regulatora prędkości zakończył działanie.")

if __name__ == '__main__':
    main()