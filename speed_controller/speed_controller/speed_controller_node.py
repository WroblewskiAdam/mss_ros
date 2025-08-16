# speed_controller/speed_controller/speed_controller_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from my_robot_interfaces.msg import StampedInt32, GpsRtk, Gear, SpeedControllerState # Zaktualizowany import
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import json
import psutil
from std_srvs.srv import SetBool
# NOWY IMPORT: Do obsługi parametrów
from rcl_interfaces.msg import SetParametersResult

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # --- Deklaracja parametrów ---
        self.declare_parameter('kp', 10.00)
        self.declare_parameter('ki', 20.00)
        # ... (reszta parametrów bez zmian)
        self.declare_parameter('v_idle', 1.3359)
        self.declare_parameter('servo_min_angle', 0)
        self.declare_parameter('servo_max_angle', 150)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('output_min', 0.0)
        self.declare_parameter('output_max', 150.0)

        # --- Pobranie parametrów ---
        self.Kp = self.get_parameter('kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('ki').get_parameter_value().double_value
        # ... (reszta pobierania parametrów bez zmian)
        self.v_idle = self.get_parameter('v_idle').get_parameter_value().double_value
        self.servo_min_angle = self.get_parameter('servo_min_angle').get_parameter_value().integer_value
        self.controller_frequency = self.get_parameter('controller_frequency').get_parameter_value().double_value
        self.output_min = self.get_parameter('output_min').get_parameter_value().double_value
        self.output_max = self.get_parameter('output_max').get_parameter_value().double_value
        self.dt_controller = 1.0 / self.controller_frequency

        # --- Zmienne regulatora ---
        self.current_speed_mps = 0.0
        self.target_speed_mps = self.v_idle
        self.integral_sum = 0.0
        self.clutch_pressed = False
        self.autopilot_enabled = False

        # --- QoS ---
        default_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        # QoS dla danych o wysokiej częstotliwości
        high_freq_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


        # --- Subskrypcje ---
        self.create_subscription(Float64, '/target_speed', self.target_speed_callback, default_qos)
        self.create_subscription(GpsRtk, '/gps_rtk_data_filtered', self.current_speed_callback, default_qos)
        self.create_subscription(Gear, '/gears', self.gear_callback, default_qos)

        # --- Publishery ---
        self.servo_command_pub = self.create_publisher(StampedInt32, '/servo/set_angle', default_qos)
        # NOWY PUBLISHER: Do danych na wykres (20 Hz)
        self.state_pub = self.create_publisher(SpeedControllerState, '/speed_controller/state', high_freq_qos)
        # NOWY PUBLISHER: Health reporting
        self.health_pub = self.create_publisher(String, '/mss/node_health/speed_controller_node', default_qos)

        # --- Serwis i Timery ---
        self.controller_timer = self.create_timer(self.dt_controller, self.controller_loop)
        self.enable_service = self.create_service(SetBool, 'speed_controller/set_enabled', self.set_enabled_callback)
        # NOWOŚĆ: Callback do dynamicznej zmiany parametrów
        self.add_on_set_parameters_callback(self.parameters_callback)
        # NOWY TIMER: Health reporting co 5 sekund
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        self.get_logger().info("Węzeł regulatora prędkości PI uruchomiony.")

    def parameters_callback(self, params):
        """Ten callback jest automatycznie wywoływany, gdy ktoś zmieni parametry węzła."""
        for param in params:
            if param.name == "kp":
                self.Kp = param.value
                self.get_logger().info(f"Zmieniono Kp na: {self.Kp}")
            elif param.name == "ki":
                self.Ki = param.value
                self.get_logger().info(f"Zmieniono Ki na: {self.Ki}")
        return SetParametersResult(successful=True)

    def set_enabled_callback(self, request, response):
        self.autopilot_enabled = request.data
        if self.autopilot_enabled:
            self.get_logger().warn("AUTOPILOT AKTYWOWANY.")
            self.integral_sum = 0.0
        else:
            self.get_logger().warn("AUTOPILOT DEZAKTYWOWANY.")
            self.set_servo_to_zero_and_wait()
        response.success = True
        return response

    def target_speed_callback(self, msg): self.target_speed_mps = msg.data if msg.data >= self.v_idle else self.v_idle
    def current_speed_callback(self, msg): self.current_speed_mps = msg.speed_mps
    def gear_callback(self, msg): self.clutch_pressed = (msg.clutch_state == 1)

    def controller_loop(self):
        saturated_control_signal = 0.0
        
        if self.autopilot_enabled and not self.clutch_pressed:
            target_speed_dev = self.target_speed_mps - self.v_idle
            current_speed_dev = self.current_speed_mps - self.v_idle
            error = target_speed_dev - current_speed_dev
            
            output_p = self.Kp * error
            output_i_potential_contribution = self.Ki * error * self.dt_controller
            
            unbounded_control_signal = output_p + self.integral_sum + output_i_potential_contribution
            if not ((unbounded_control_signal >= self.output_max and error > 0) or (unbounded_control_signal <= self.output_min and error < 0)):
                self.integral_sum += output_i_potential_contribution

            final_unbounded_signal = output_p + self.integral_sum
            saturated_control_signal = max(self.output_min, min(self.output_max, final_unbounded_signal))
        else:
            self.integral_sum = 0.0
            saturated_control_signal = self.servo_min_angle

        # Publikacja komendy do serwa TYLKO gdy autopilot włączony
        if self.autopilot_enabled:
            servo_msg = StampedInt32()
            servo_msg.header.stamp = self.get_clock().now().to_msg()
            servo_msg.data = int(round(saturated_control_signal))
            self.servo_command_pub.publish(servo_msg)

        # Publikacja danych na wykres
        state_msg = SpeedControllerState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.setpoint_speed = float(self.target_speed_mps)
        state_msg.current_speed = float(self.current_speed_mps)
        state_msg.control_output = float(saturated_control_signal)
        self.state_pub.publish(state_msg)

    def set_servo_to_zero_and_wait(self):
        # ... (bez zmian)
        pass

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            # Zbierz metryki systemu
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory = psutil.virtual_memory()
            
            # Sprawdź błędy i ostrzeżenia
            errors = []
            warnings = []
            
            # Sprawdź czy węzeł ma problemy
            if not self.autopilot_enabled:
                warnings.append("Autopilot wyłączony")
            
            if self.clutch_pressed:
                warnings.append("Sprzęgło wciśnięte")
            
            # Sprawdź czy dane są aktualne
            current_time = time.time()
            if hasattr(self, 'last_speed_update'):
                time_since_speed = current_time - self.last_speed_update
                if time_since_speed > 5.0:
                    warnings.append(f"Brak aktualizacji prędkości przez {time_since_speed:.1f}s")
            
            # Stwórz wiadomość health
            health_data = {
                'status': 'running',
                'last_update': current_time,
                'errors': errors,
                'warnings': warnings,
                'metrics': {
                    'cpu_usage': cpu_percent,
                    'memory_usage': memory.percent,
                    'target_speed': self.target_speed_mps,
                    'current_speed': self.current_speed_mps,
                    'autopilot_enabled': self.autopilot_enabled,
                    'clutch_pressed': self.clutch_pressed
                }
            }
            
            health_msg = String()
            health_msg.data = json.dumps(health_data)
            self.health_pub.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd podczas publikacji health: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
