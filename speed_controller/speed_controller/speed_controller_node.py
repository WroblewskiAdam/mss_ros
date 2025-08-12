# speed_controller/speed_controller/speed_controller_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from my_robot_interfaces.msg import StampedInt32, GpsRtk, Gear 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter import Parameter
# NOWY IMPORT: Serwis do włączania/wyłączania
from std_srvs.srv import SetBool

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # --- Deklaracja parametrów (bez zmian) ---
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('kp', 10.00, ParameterDescriptor(description='Wzmocnienie proporcjonalne (Kp) regulatora PI'))
        self.declare_parameter('ki', 20.00, ParameterDescriptor(description='Wzmocnienie całkujące (Ki) regulatora PI'))
        self.declare_parameter('v_idle', 1.3359)
        self.declare_parameter('servo_min_angle', 0)
        self.declare_parameter('servo_max_angle', 150)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('target_speed_topic', '/target_speed')
        self.declare_parameter('current_speed_topic', '/gps_rtk_data_filtered')
        self.declare_parameter('servo_command_topic', '/servo/set_angle')
        self.declare_parameter('gear_topic', '/gears', ParameterDescriptor(description='Temat z informacją o stanie biegów i sprzęgła'))
        self.declare_parameter('output_min', 0.0)
        self.declare_parameter('output_max', 150.0)

        # --- Pobranie parametrów (bez zmian) ---
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
        gear_topic_name = self.get_parameter('gear_topic').get_parameter_value().string_value
        self.dt_controller = 1.0 / self.controller_frequency

        # --- Zmienne regulatora ---
        self.current_speed_mps = 0.0
        self.target_speed_mps = self.v_idle
        self.integral_sum = 0.0
        self.clutch_pressed = False
        # NOWA ZMIENNA: Stan autopilota
        self.autopilot_enabled = False

        # --- QoS (bez zmian) ---
        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subskrypcje (bez zmian) ---
        self.target_speed_sub = self.create_subscription(Float64, target_speed_topic_name, self.target_speed_callback, default_qos)
        self.current_speed_sub = self.create_subscription(GpsRtk, current_speed_topic_name, self.current_speed_callback, default_qos)
        self.gear_sub = self.create_subscription(Gear, gear_topic_name, self.gear_callback, default_qos)

        # --- Publisher (bez zmian) ---
        self.servo_command_pub = self.create_publisher(StampedInt32, servo_command_topic_name, default_qos)

        # --- Timer dla pętli regulatora (bez zmian) ---
        self.controller_timer = self.create_timer(self.dt_controller, self.controller_loop)

        # --- NOWY SERWIS: Do włączania/wyłączania autopilota ---
        self.enable_service = self.create_service(SetBool, 'speed_controller/set_enabled', self.set_enabled_callback)

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info(f"Węzeł regulatora prędkości PI uruchomiony. Autopilot domyślnie WYŁĄCZONY.")
        self.get_logger().info(f"  Aby aktywować, użyj serwisu: /speed_controller/set_enabled")

    # === NOWY CALLBACK: Obsługa serwisu ===
    def set_enabled_callback(self, request, response):
        """Callback do włączania/wyłączania autopilota."""
        self.autopilot_enabled = request.data
        if self.autopilot_enabled:
            self.get_logger().warn("AUTOPILOT AKTYWOWANY przez operatora.")
            self.integral_sum = 0.0 # Reset całki, aby uniknąć skoku po włączeniu
        else:
            self.get_logger().warn("AUTOPILOT DEZAKTYWOWANY przez operatora.")
            # Bezpiecznie ustawiamy serwo na zero po wyłączeniu
            self.set_servo_to_zero_and_wait()

        response.success = True
        response.message = f"Autopilot set to: {self.autopilot_enabled}"
        return response

    def parameters_callback(self, params):
        # ... (bez zmian)
        from rcl_interfaces.msg import SetParametersResult
        successful = True
        reason = ""
        for param in params:
            if param.name == "kp":
                self.Kp = param.value
                self.get_logger().info(f"Zmieniono Kp na: {self.Kp}")
            elif param.name == "ki":
                self.Ki = param.value
                self.get_logger().info(f"Zmieniono Ki na: {self.Ki}")
        return SetParametersResult(successful=successful, reason=reason)

    def target_speed_callback(self, msg):
        # ... (bez zmian)
        if msg.data < self.v_idle:
            self.target_speed_mps = self.v_idle
        else:
            self.target_speed_mps = msg.data

    def current_speed_callback(self, msg):
        # ... (bez zmian)
        self.current_speed_mps = msg.speed_mps

    def gear_callback(self, msg):
        # ... (bez zmian)
        self.clutch_pressed = (msg.clutch_state == 1)

    def controller_loop(self):
        # === ZMODYFIKOWANA LOGIKA BEZPIECZEŃSTWA ===
        # Regulator działa TYLKO, gdy autopilot jest włączony ORAZ sprzęgło nie jest wciśnięte
        if not self.autopilot_enabled or self.clutch_pressed:
            if not self.autopilot_enabled:
                self.get_logger().info('Autopilot wyłączony. Regulator nie pracuje.', throttle_duration_sec=5)
            if self.clutch_pressed:
                self.get_logger().warn('Sprzęgło wciśnięte! Regulator prędkości DEZAKTYWOWANY.', throttle_duration_sec=2)

            self.integral_sum = 0.0
            servo_msg = StampedInt32()
            servo_msg.header.stamp = self.get_clock().now().to_msg()
            servo_msg.data = int(self.servo_min_angle)
            self.servo_command_pub.publish(servo_msg)
            return
        # ==================================

        # Poniższy kod regulatora PI pozostaje bez zmian
        target_speed_dev = self.target_speed_mps - self.v_idle
        current_speed_dev = self.current_speed_mps - self.v_idle
        error = target_speed_dev - current_speed_dev
        
        output_p = self.Kp * error
        output_i_potential_contribution = self.Ki * error * self.dt_controller
        
        unbounded_control_signal = output_p + self.integral_sum + output_i_potential_contribution

        if ( (self.integral_sum + output_p >= self.output_max and error > 0) or \
             (self.integral_sum + output_p <= self.output_min and error < 0) ) and \
           ( (unbounded_control_signal >= self.output_max) or (unbounded_control_signal <= self.output_min) ):
            pass
        else:
            self.integral_sum += output_i_potential_contribution

        final_unbounded_signal = output_p + self.integral_sum
        saturated_control_signal = max(self.output_min, min(self.output_max, final_unbounded_signal))
        
        servo_angle_final = saturated_control_signal
        servo_msg = StampedInt32()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.data = int(round(servo_angle_final))
        self.servo_command_pub.publish(servo_msg)
        
        self.get_logger().debug(
            f"Kp:{self.Kp:.2f} Ki:{self.Ki:.2f} "
            f"TgtDev:{target_speed_dev:.2f} CurDev:{current_speed_dev:.2f} Err:{error:.2f} "
            f"P_out:{output_p:.2f} I_sum:{self.integral_sum:.2f} Unbnd:{final_unbounded_signal:.2f} ServoCmd:{servo_angle_final:.1f}"
        )

    def set_servo_to_zero_and_wait(self):
        # ... (bez zmian)
        self.get_logger().info("Ustawianie serwa na 0 stopni...")
        servo_msg = StampedInt32()
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.data = int(self.servo_min_angle)
        for _ in range(10): 
            self.servo_command_pub.publish(servo_msg)
            time.sleep(0.05)
        time.sleep(0.5)

def main(args=None):
    # ... (główna funkcja bez zmian)
    rclpy.init(args=args)
    node = None
    try:
        node = SpeedControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.set_servo_to_zero_and_wait()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
