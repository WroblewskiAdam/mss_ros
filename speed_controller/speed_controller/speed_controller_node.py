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
from rcl_interfaces.srv import SetParameters

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # --- Deklaracja parametrów ---
        # Parametr kontrolny main_gear
        self.declare_parameter('main_gear', 1)
        
        # Podstawowe nastawy regulatora (używane gdy main_gear=2 lub jako fallback)
        self.declare_parameter('kp', 20.50)
        self.declare_parameter('ki', 40.50)
        self.declare_parameter('kd', 0.00)
        
        # # Nastawy dla półbiegu 1 (sub_gear_1)
        # self.declare_parameter('sub_gear_1_kp', 35.0)
        # self.declare_parameter('sub_gear_1_ki', 70.0)
        # self.declare_parameter('sub_gear_1_kd', 0.0)
        
        # # Nastawy dla półbiegu 2 (sub_gear_2)
        # self.declare_parameter('sub_gear_2_kp', 35.0)
        # self.declare_parameter('sub_gear_2_ki', 70.0)
        # self.declare_parameter('sub_gear_2_kd', 0.0)
        
        # # Nastawy dla półbiegu 3 (sub_gear_3)
        # self.declare_parameter('sub_gear_3_kp', 30.00)
        # self.declare_parameter('sub_gear_3_ki', 60.00)
        # self.declare_parameter('sub_gear_3_kd', 0.00)
        
        # # Nastawy dla półbiegu 4 (sub_gear_4)
        # self.declare_parameter('sub_gear_4_kp', 30.0)
        # self.declare_parameter('sub_gear_4_ki', 60.0)
        # self.declare_parameter('sub_gear_4_kd', 0.00)

         # Nastawy dla półbiegu 1 (sub_gear_1)
        self.declare_parameter('sub_gear_1_kp', 27.2)
        self.declare_parameter('sub_gear_1_ki', 81.5)
        self.declare_parameter('sub_gear_1_kd', 0.0)
        
        # Nastawy dla półbiegu 2 (sub_gear_2)
        self.declare_parameter('sub_gear_2_kp', 27.25)
        self.declare_parameter('sub_gear_2_ki', 76.0)
        self.declare_parameter('sub_gear_2_kd', 0.0)
        
        # Nastawy dla półbiegu 3 (sub_gear_3)
        self.declare_parameter('sub_gear_3_kp', 23.5)
        self.declare_parameter('sub_gear_3_ki', 60.25)
        self.declare_parameter('sub_gear_3_kd', 0.00)
        
        # Nastawy dla półbiegu 4 (sub_gear_4)
        self.declare_parameter('sub_gear_4_kp', 28.75)
        self.declare_parameter('sub_gear_4_ki', 56.25)
        self.declare_parameter('sub_gear_4_kd', 0.00)
        
        # Pozostałe parametry bez zmian
        self.declare_parameter('v_idle', 0.7)
        self.declare_parameter('servo_min_angle', 0)
        self.declare_parameter('servo_max_angle', 150)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('output_min', 0.0)
        self.declare_parameter('output_max', 150.0)

        # --- Pobranie parametrów ---
        self.main_gear = self.get_parameter('main_gear').get_parameter_value().integer_value
        
        # Podstawowe nastawy regulatora (używane gdy main_gear=2 lub jako fallback)
        self.Kp = self.get_parameter('kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('kd').get_parameter_value().double_value
        
        # Nastawy dla sub-gear (półbiegów)
        self.sub_gear_params = {
            1: {
                'kp': self.get_parameter('sub_gear_1_kp').get_parameter_value().double_value,
                'ki': self.get_parameter('sub_gear_1_ki').get_parameter_value().double_value,
                'kd': self.get_parameter('sub_gear_1_kd').get_parameter_value().double_value
            },
            2: {
                'kp': self.get_parameter('sub_gear_2_kp').get_parameter_value().double_value,
                'ki': self.get_parameter('sub_gear_2_ki').get_parameter_value().double_value,
                'kd': self.get_parameter('sub_gear_2_kd').get_parameter_value().double_value
            },
            3: {
                'kp': self.get_parameter('sub_gear_3_kp').get_parameter_value().double_value,
                'ki': self.get_parameter('sub_gear_3_ki').get_parameter_value().double_value,
                'kd': self.get_parameter('sub_gear_3_kd').get_parameter_value().double_value
            },
            4: {
                'kp': self.get_parameter('sub_gear_4_kp').get_parameter_value().double_value,
                'ki': self.get_parameter('sub_gear_4_ki').get_parameter_value().double_value,
                'kd': self.get_parameter('sub_gear_4_kd').get_parameter_value().double_value
            }
        }
        
        # Pozostałe parametry bez zmian
        self.v_idle = self.get_parameter('v_idle').get_parameter_value().double_value
        self.servo_min_angle = self.get_parameter('servo_min_angle').get_parameter_value().integer_value
        self.servo_max_angle = self.get_parameter('servo_max_angle').get_parameter_value().integer_value
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
        self.current_gear = 1  # Aktualny bieg (1-4)
        
        # --- FEEDFORWARD: Współczynniki wielomianów 3. stopnia dla inicjalizacji integratora ---
        # Format: [a3, a2, a1, a0] dla wielomianu: angle = a3*s³ + a2*s² + a1*s + a0
        # Zestawy współczynników w zależności od main_gear
        self.feedforward_coefficients_by_main = {
            1: {
                1: [10.186676212107146, -13.54134634347516, 82.42113487265581, -62.755627],
                2: [6.955916612380406, -14.248646239342783, 78.86743745241643, -66.27996103199986],
                3: [3.7992054666586084, -6.261805895905663, 55.41186525541788, -56.13024547701927],
                4: [2.8861966188761903, -6.538251862577834, 48.70783078668607, -55.59988787686971]
            },
            2: {
                1: [0.5873549785294848, 13.40037863553026, 14.653379751052785, -34.52735487782364],
                2: [1.4600581419748402, -0.7755560817553078, 38.18436232184589, -57.7959979072928],
                3: [3.821886722815732, -22.385212129313977, 85.72752018970289, -99.19677542430101],
                4: [0.32581512631611265, 0.5479366362622122, 17.692958476521984, -45.46624605117514]
            }
        }

        # Aktywny zestaw współczynników odpowiadający aktualnemu main_gear
        self.feedforward_coefficients = self.feedforward_coefficients_by_main.get(self.main_gear, self.feedforward_coefficients_by_main[2])

        # --- QoS ---
        default_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        # QoS dla danych o wysokiej częstotliwości
        high_freq_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


        # --- Subskrypcje ---
        self.create_subscription(Float64, '/target_speed', self.target_speed_callback, default_qos)
        self.create_subscription(GpsRtk, '/gps_rtk_data/tractor_filtered', self.current_speed_callback, default_qos)
        self.create_subscription(Gear, '/gears', self.gear_callback, default_qos)

        # --- Publishery ---
        self.servo_command_pub = self.create_publisher(StampedInt32, '/servo/set_angle', default_qos)
        # NOWY PUBLISHER: Do danych na wykres (20 Hz) - RELIABLE dla zgodności z loggerem
        self.state_pub = self.create_publisher(SpeedControllerState, '/speed_controller/state', default_qos)
        # NOWY PUBLISHER: Health reporting
        self.health_pub = self.create_publisher(String, '/mss/node_health/speed_controller_node', default_qos)

        # --- Serwis i Timery ---
        self.controller_timer = self.create_timer(self.dt_controller, self.controller_loop)
        self.enable_service = self.create_service(SetBool, 'speed_controller/set_enabled', self.set_enabled_callback)
        # NOWY SERWIS: Do ręcznego ustawiania parametrów
        self.set_parameters_service = self.create_service(SetParameters, 'speed_controller_node/set_parameters', self.set_parameters_service_callback)
        # NOWOŚĆ: Callback do dynamicznej zmiany parametrów
        self.add_on_set_parameters_callback(self.parameters_callback)
        # NOWY TIMER: Health reporting co 5 sekund
        self.health_timer = self.create_timer(5.0, self.publish_health)
        # USUNIĘTO: Timer publikowania parametrów co 5 sekund
        
        self.get_logger().info("Węzeł regulatora prędkości PI z feedforward uruchomiony.")
        self.get_logger().info(f"PARAMETRY GEAR: main_gear={self.main_gear}")
        if self.main_gear == 1:
            self.get_logger().info("MODE: Parametry różne dla każdego półbiegu")
            for gear_num, params in self.sub_gear_params.items():
                self.get_logger().info(f"  Półbieg {gear_num}: kp={params['kp']}, ki={params['ki']}, kd={params['kd']}")
        elif self.main_gear == 2:
            self.get_logger().info(f"MODE: Stałe parametry dla wszystkich półbiegów: kp={self.Kp}, ki={self.Ki}, kd={self.Kd}")
        self.get_logger().info(f"FEEDFORWARD: main_gear={self.main_gear}, wielomiany 3. stopnia załadowane dla biegów: {list(self.feedforward_coefficients.keys())}")
        self.get_logger().info("Feedforward używany TYLKO do inicjalizacji integratora przy włączaniu autopilota")
        self.log_current_feedforward_coeffs()

    def update_parameter_and_variable(self, param_name, value, variable_name):
        """Aktualizuje zmienną wewnętrzną."""
        try:
            # Aktualizuj zmienną wewnętrzną
            setattr(self, variable_name, value)
            self.get_logger().info(f"Zaktualizowano {param_name} na: {value}")
            return True
        except Exception as e:
            self.get_logger().error(f"Błąd podczas aktualizacji {param_name}: {e}")
            return False

    def get_current_pid_params(self):
        """
        Zwraca aktualne parametry PID na podstawie main_gear i current_gear.
        
        Returns:
            dict: {'kp': float, 'ki': float, 'kd': float}
        """
        if self.main_gear == 1:
            # Gdy main_gear=1, używaj parametrów specyficznych dla półbiegu
            if self.current_gear in self.sub_gear_params:
                params = self.sub_gear_params[self.current_gear]
                self.get_logger().debug(f"Używanie parametrów sub_gear {self.current_gear}: kp={params['kp']}, ki={params['ki']}, kd={params['kd']}")
                return params
            else:
                # Fallback na podstawowe nastawy jeśli półbieg nieznany
                self.get_logger().warn(f"Nieznany półbieg {self.current_gear}, używanie podstawowych nastaw")
                return {'kp': self.Kp, 'ki': self.Ki, 'kd': self.Kd}
        elif self.main_gear == 2:
            # Gdy main_gear=2, wszystkie półbiegi używają stałych nastaw
            self.get_logger().debug(f"Używanie podstawowych nastaw dla wszystkich półbiegów: kp={self.Kp}, ki={self.Ki}, kd={self.Kd}")
            return {'kp': self.Kp, 'ki': self.Ki, 'kd': self.Kd}
        else:
            # Fallback dla nieznanych wartości main_gear
            self.get_logger().warn(f"Nieznana wartość main_gear={self.main_gear}, używanie podstawowych nastaw")
            return {'kp': self.Kp, 'ki': self.Ki, 'kd': self.Kd}

    def parameters_callback(self, params):
        """Ten callback jest automatycznie wywoływany, gdy ktoś zmieni parametry węzła."""
        for param in params:
            if param.name == "main_gear":
                # Zaktualizuj main_gear i aktywny zestaw współczynników feedforward
                if self.update_parameter_and_variable('main_gear', param.value, 'main_gear'):
                    self.feedforward_coefficients = self.feedforward_coefficients_by_main.get(
                        self.main_gear,
                        self.feedforward_coefficients_by_main[2]
                    )
                    self.get_logger().info(
                        f"FEEDFORWARD: przełączenie na main_gear={self.main_gear}, dostępne biegi: {list(self.feedforward_coefficients.keys())}"
                    )
                    self.log_current_feedforward_coeffs()
            elif param.name == "kp":
                self.update_parameter_and_variable('kp', param.value, 'Kp')
            elif param.name == "ki":
                self.update_parameter_and_variable('ki', param.value, 'Ki')
            elif param.name == "kd":
                self.update_parameter_and_variable('kd', param.value, 'Kd')
            elif param.name.startswith("sub_gear_"):
                # Obsługa parametrów sub_gear
                gear_num = None
                param_type = None
                if param.name.startswith("sub_gear_1_"):
                    gear_num = 1
                    if param.name.endswith("_kp"):
                        param_type = "kp"
                    elif param.name.endswith("_ki"):
                        param_type = "ki"
                    elif param.name.endswith("_kd"):
                        param_type = "kd"
                elif param.name.startswith("sub_gear_2_"):
                    gear_num = 2
                    if param.name.endswith("_kp"):
                        param_type = "kp"
                    elif param.name.endswith("_ki"):
                        param_type = "ki"
                    elif param.name.endswith("_kd"):
                        param_type = "kd"
                elif param.name.startswith("sub_gear_3_"):
                    gear_num = 3
                    if param.name.endswith("_kp"):
                        param_type = "kp"
                    elif param.name.endswith("_ki"):
                        param_type = "ki"
                    elif param.name.endswith("_kd"):
                        param_type = "kd"
                elif param.name.startswith("sub_gear_4_"):
                    gear_num = 4
                    if param.name.endswith("_kp"):
                        param_type = "kp"
                    elif param.name.endswith("_ki"):
                        param_type = "ki"
                    elif param.name.endswith("_kd"):
                        param_type = "kd"
                
                if gear_num is not None and param_type is not None:
                    self.sub_gear_params[gear_num][param_type] = param.value
                    self.get_logger().info(f"Zaktualizowano parametr sub_gear_{gear_num}_{param_type} na: {param.value}")
        return SetParametersResult(successful=True)

    def set_parameters_service_callback(self, request, response):
        """Serwis do ustawiania parametrów regulatora."""
        try:
            success_count = 0
            total_params = len(request.parameters)
            
            for param in request.parameters:
                param_name = param.name
                param_value = param.value.double_value
                
                if param_name == 'kp':
                    if self.update_parameter_and_variable('kp', param_value, 'Kp'):
                        success_count += 1
                elif param_name == 'ki':
                    if self.update_parameter_and_variable('ki', param_value, 'Ki'):
                        success_count += 1
                elif param_name == 'kd':
                    if self.update_parameter_and_variable('kd', param_value, 'Kd'):
                        success_count += 1
                else:
                    self.get_logger().warn(f"Nieznany parametr: {param_name}")
            
            # Przygotuj odpowiedź
            response.results = []
            for param in request.parameters:
                result = SetParametersResult()
                result.successful = param.name in ['kp', 'ki', 'kd'] and success_count > 0
                response.results.append(result)
            
            if success_count == total_params:
                self.get_logger().info(f"Wszystkie parametry zaktualizowane pomyślnie: kp={self.Kp}, ki={self.Ki}, kd={self.Kd}")
            else:
                self.get_logger().warn(f"Zaktualizowano {success_count}/{total_params} parametrów")
                
            return response
            
        except Exception as e:
            self.get_logger().error(f"Błąd w serwisie ustawiania parametrów: {e}")
            response.results = []
            for param in request.parameters:
                result = SetParametersResult()
                result.successful = False
                response.results.append(result)
            return response

    def set_enabled_callback(self, request, response):
        self.autopilot_enabled = request.data
        if self.autopilot_enabled:
            self.get_logger().info("REGULATOR PRĘDKOŚCI WŁĄCZONY.")
            
            # --- FEEDFORWARD: Inicjalizacja integratora na podstawie wielomianu 3. stopnia ---
            # Zamiast resetować integrator na 0, ustawiamy go na wartość z charakterystyki statycznej
            # Używamy AKTUALNEJ prędkości ciągnika (już zsynchronizowanej z sieczkarnią)
            # Dzięki temu unikamy szarpnięcia przy włączaniu autopilota
            integrator_init_value = self.calculate_integrator_initialization(self.current_speed_mps)
            self.integral_sum = integrator_init_value
            
            self.get_logger().info(f"Integrator zainicjalizowany wartością: {integrator_init_value:.1f}° (na podstawie aktualnej prędkości: {self.current_speed_mps:.2f}m/s)")
        else:
            self.get_logger().info("REGULATOR PRĘDKOŚCI WYŁĄCZONY.")
            self.integral_sum = 0.0  # Reset integratora przy wyłączaniu
            self.set_servo_to_zero_and_wait()
        response.success = True
        return response

    def target_speed_callback(self, msg): self.target_speed_mps = msg.data if msg.data >= self.v_idle else self.v_idle
    def current_speed_callback(self, msg): self.current_speed_mps = msg.speed_mps
    def gear_callback(self, msg): 
        self.clutch_pressed = (msg.clutch_state == 1)
        previous_gear = self.current_gear
        self.current_gear = msg.gear  # Aktualizuj aktualny bieg
        
        # Logowanie zmian półbiegu gdy main_gear=1
        if (self.main_gear == 1 and previous_gear != msg.gear and msg.gear != 0):
            current_params = self.get_current_pid_params()
            self.get_logger().info(f"ZMIANA PÓŁBIEGU: {previous_gear} → {msg.gear}")
            self.get_logger().info(f"NOWE PARAMETRY PID dla półbiegu {msg.gear}: kp={current_params['kp']}, ki={current_params['ki']}, kd={current_params['kd']}")

    def calculate_integrator_initialization(self, current_speed_mps):
        """
        Oblicza wartość inicjalizacji integratora na podstawie wielomianu 3. stopnia.
        Używane tylko przy włączaniu autopilota aby uniknąć szarpnięcia.
        
        Args:
            current_speed_mps: Aktualna prędkość ciągnika w m/s (już zsynchronizowana z sieczkarnią)
            
        Returns:
            float: Wartość inicjalizacji integratora
        """
        if self.current_gear not in self.feedforward_coefficients:
            self.get_logger().warn(f"Brak współczynników feedforward dla biegu {self.current_gear}")
            return 0.0
        
        # Pobierz współczynniki dla aktualnego biegu
        coeffs = self.feedforward_coefficients[self.current_gear]
        a3, a2, a1, a0 = coeffs
        
        # Oblicz wielomian 3. stopnia: angle = a3*s³ + a2*s² + a1*s + a0
        # gdzie s to prędkość względem prędkości jałowej
        s = current_speed_mps
        
        # Oblicz wartość wielomianu
        feedforward_angle = a3 * (s**3) + a2 * (s**2) + a1 * s + a0
        
        # Ogranicz do zakresu serwa
        feedforward_angle = max(self.servo_min_angle, min(self.servo_max_angle, feedforward_angle))
        
        self.get_logger().info(f"INICJALIZACJA INTEGRATORA: bieg={self.current_gear}, aktualna_prędkość={current_speed_mps:.2f}m/s, s={s:.2f}, angle={feedforward_angle:.1f}°")
        
        return feedforward_angle

    def log_current_feedforward_coeffs(self):
        """Loguje aktualnie używane współczynniki feedforward dla każdego półbiegu."""
        try:
            self.get_logger().info(f"FEEDFORWARD: aktywny profil main_gear={self.main_gear}")
            for gear_num in sorted(self.feedforward_coefficients.keys()):
                coeffs = self.feedforward_coefficients.get(gear_num, None)
                if coeffs is None:
                    continue
                a3, a2, a1, a0 = coeffs
                self.get_logger().info(
                    f"  bieg {gear_num}: a3={a3:.6f}, a2={a2:.6f}, a1={a1:.6f}, a0={a0:.6f}"
                )
        except Exception as e:
            self.get_logger().warn(f"Nie udało się zalogować współczynników feedforward: {e}")

    def controller_loop(self):
        saturated_control_signal = 0.0
        
        if self.autopilot_enabled and not self.clutch_pressed:
            # Pobierz aktualne parametry PID na podstawie main_gear i current_gear
            current_params = self.get_current_pid_params()
            current_kp = current_params['kp']
            current_ki = current_params['ki']
            current_kd = current_params['kd']
            
            target_speed_dev = self.target_speed_mps - self.v_idle
            current_speed_dev = self.current_speed_mps - self.v_idle
            error = target_speed_dev - current_speed_dev
            
            output_p = current_kp * error
            output_i_potential_contribution = current_ki * error * self.dt_controller
            
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
