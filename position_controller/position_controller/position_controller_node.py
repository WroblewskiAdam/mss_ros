#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64, String, Bool
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from my_robot_interfaces.msg import GpsRtk, DistanceMetrics
import time
import math

class PositionControllerNode(Node):
    def __init__(self):
        super().__init__('position_controller_node')
        
        # --- Parametry konfiguracyjne ---
        self.declare_parameter('target_distance', 0.0)  # m - docelowa odległość wzdłużna
        self.declare_parameter('position_tolerance', 1.0)  # m - tolerancja pozycji
        self.declare_parameter('speed_tolerance', 0.41)  # m/s - tolerancja prędkości (1 km/h)
        self.declare_parameter('Kp', 1.0)  # wzmocnienie proporcjonalne
        self.declare_parameter('Ki', 0.1)  # wzmocnienie całkujące
        self.declare_parameter('min_speed', 0.5)  # m/s - minimalna prędkość
        self.declare_parameter('max_speed', 8.0)  # m/s - maksymalna prędkość
        self.declare_parameter('max_acceleration', 0.5)  # m/s² - maksymalne przyspieszenie
        self.declare_parameter('gps_timeout', 2.0)  # s - timeout sygnału GPS
        self.declare_parameter('control_frequency', 10.0)  # Hz - częstotliwość regulacji
        
        # Pobierz parametry
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.speed_tolerance = self.get_parameter('speed_tolerance').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.gps_timeout = self.get_parameter('gps_timeout').get_parameter_value().double_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # --- Zmienne stanu ---
        self.is_enabled = False  # Stan włączania/wyłączania autopilota
        self.activation_conditions_checked = False  # Czy warunki aktywacji zostały już sprawdzone
        self.current_distance_longitudinal = 0.0  # Aktualna odległość wzdłużna
        self.tractor_speed = 0.0  # Prędkość ciągnika
        self.harvester_speed = 0.0  # Prędkość sieczkarni
        self.integral_error = 0.0  # Błąd całkujący dla regulatora PI
        self.last_target_speed = 0.0  # Ostatnia zadana prędkość
        self.last_control_time = time.time()  # Czas ostatniej regulacji
        
        # --- Timestamps dla timeoutów ---
        self.last_distance_update = time.time()
        self.last_tractor_gps_update = time.time()
        self.last_harvester_gps_update = time.time()
        
        # --- QoS ---
        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- Subskrypcje ---
        self.create_subscription(
            DistanceMetrics, 
            '/distance_metrics', 
            self.distance_callback, 
            default_qos
        )
        
        self.create_subscription(
            GpsRtk, 
            '/gps_rtk_data/tractor_filtered', 
            self.tractor_gps_callback, 
            default_qos
        )
        
        self.create_subscription(
            GpsRtk, 
            '/gps_rtk_data/chopper_filtered', 
            self.harvester_gps_callback, 
            default_qos
        )
        
        # Subskrypcja pozycji zadanej z web interface
        self.create_subscription(
            Float64, 
            '/target_position', 
            self.target_position_callback, 
            default_qos
        )
        
        # --- Publikacje ---
        self.target_speed_publisher = self.create_publisher(Float64, '/target_speed', default_qos)
        self.status_publisher = self.create_publisher(String, '/autopilot/status', default_qos)
        self.health_publisher = self.create_publisher(String, '/position_controller/health', default_qos)
        
        # --- Serwisy ---
        self.set_enabled_service = self.create_service(
            SetBool, 
            '/position_controller/set_enabled', 
            self.set_enabled_callback
        )
        
        # NOWY SERWIS: Do ręcznego ustawiania parametrów
        self.set_parameters_service = self.create_service(
            SetParameters, 
            'position_controller_node/set_parameters', 
            self.set_parameters_service_callback
        )
        
        # NOWOŚĆ: Callback do dynamicznej zmiany parametrów
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # --- Timery ---
        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)
        self.health_timer = self.create_timer(5.0, self.publish_health)
        
        # --- Logowanie ---
        self.get_logger().info("=== POSITION CONTROLLER NODE STARTED ===")
        self.get_logger().info(f"Docelowa odległość: {self.target_distance} m")
        self.get_logger().info(f"Tolerancja pozycji: ±{self.position_tolerance} m")
        self.get_logger().info(f"Tolerancja prędkości: ±{self.speed_tolerance} m/s")
        self.get_logger().info(f"Nastawy PI: Kp={self.Kp}, Ki={self.Ki}")
        self.get_logger().info(f"Autopilot domyślnie WYŁĄCZONY")

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

    def parameters_callback(self, params):
        """Ten callback jest automatycznie wywoływany, gdy ktoś zmieni parametry węzła."""
        for param in params:
            if param.name == "Kp":
                self.update_parameter_and_variable('Kp', param.value, 'Kp')
            elif param.name == "Ki":
                self.update_parameter_and_variable('Ki', param.value, 'Ki')
            elif param.name == "position_tolerance":
                self.update_parameter_and_variable('position_tolerance', param.value, 'position_tolerance')
            elif param.name == "speed_tolerance":
                self.update_parameter_and_variable('speed_tolerance', param.value, 'speed_tolerance')
            elif param.name == "target_distance":
                self.update_parameter_and_variable('target_distance', param.value, 'target_distance')
        return SetParametersResult(successful=True)

    def set_parameters_service_callback(self, request, response):
        """Serwis do ustawiania parametrów regulatora pozycji."""
        try:
            success_count = 0
            total_params = len(request.parameters)
            
            for param in request.parameters:
                param_name = param.name
                param_value = param.value.double_value
                
                if param_name == 'Kp':
                    if self.update_parameter_and_variable('Kp', param_value, 'Kp'):
                        success_count += 1
                elif param_name == 'Ki':
                    if self.update_parameter_and_variable('Ki', param_value, 'Ki'):
                        success_count += 1
                elif param_name == 'position_tolerance':
                    if self.update_parameter_and_variable('position_tolerance', param_value, 'position_tolerance'):
                        success_count += 1
                elif param_name == 'speed_tolerance':
                    if self.update_parameter_and_variable('speed_tolerance', param_value, 'speed_tolerance'):
                        success_count += 1
                elif param_name == 'target_distance':
                    if self.update_parameter_and_variable('target_distance', param_value, 'target_distance'):
                        success_count += 1
                else:
                    self.get_logger().warn(f"Nieznany parametr: {param_name}")
            
            # Przygotuj odpowiedź
            response.results = []
            for param in request.parameters:
                result = SetParametersResult()
                result.successful = param.name in ['Kp', 'Ki', 'position_tolerance', 'speed_tolerance', 'target_distance'] and success_count > 0
                response.results.append(result)
            
            if success_count == total_params:
                self.get_logger().info(f"Wszystkie parametry zaktualizowane pomyślnie: Kp={self.Kp}, Ki={self.Ki}, position_tolerance={self.position_tolerance}, speed_tolerance={self.speed_tolerance}, target_distance={self.target_distance}")
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

    def distance_callback(self, msg):
        """Odbiera dane o odległościach względnych."""
        self.last_distance_update = time.time()
        self.current_distance_longitudinal = msg.distance_longitudinal
        
        if self.is_enabled:
            self.get_logger().debug(f"Odległość wzdłużna: {self.current_distance_longitudinal:.2f} m")

    def tractor_gps_callback(self, msg):
        """Odbiera dane GPS ciągnika (filtrowane)."""
        self.last_tractor_gps_update = time.time()
        self.tractor_speed = msg.speed_mps
        
        if self.is_enabled:
            self.get_logger().debug(f"Prędkość ciągnika: {self.tractor_speed:.2f} m/s")

    def harvester_gps_callback(self, msg):
        """Odbiera dane GPS sieczkarni."""
        self.last_harvester_gps_update = time.time()
        self.harvester_speed = msg.speed_mps
        
        if self.is_enabled:
            self.get_logger().debug(f"Prędkość sieczkarni: {self.harvester_speed:.2f} m/s")

    def target_position_callback(self, msg):
        """Odbiera pozycję zadaną z web interface."""
        self.target_distance = msg.data
        self.get_logger().info(f"Otrzymano nową pozycję zadaną: {self.target_distance} m")

    def set_enabled_callback(self, request, response):
        """Serwis do włączania/wyłączania autopilota."""
        self.is_enabled = request.data
        
        if self.is_enabled:
            self.get_logger().info("Autopilot WŁĄCZONY")
            # Reset integratora przy włączeniu
            self.integral_error = 0.0
            # Reset flagi warunków aktywacji - będą sprawdzone przy pierwszym uruchomieniu
            self.activation_conditions_checked = False
        else:
            self.get_logger().info("Autopilot WYŁĄCZONY")
            # Reset integratora przy wyłączeniu
            self.integral_error = 0.0
            # Reset flagi warunków aktywacji
            self.activation_conditions_checked = False
        
        response.success = True
        response.message = f"Autopilot {'włączony' if self.is_enabled else 'wyłączony'}"
        return response

    def check_activation_conditions(self):
        """Sprawdza warunki aktywacji autopilota."""
        # Sprawdź timeout danych
        current_time = time.time()
        if (current_time - self.last_distance_update > self.gps_timeout or
            current_time - self.last_tractor_gps_update > self.gps_timeout or
            current_time - self.last_harvester_gps_update > self.gps_timeout):
            return False, "Timeout danych GPS"
        
        # Sprawdź warunki pozycji i prędkości
        position_ok = abs(self.current_distance_longitudinal) <= self.position_tolerance
        speed_difference = abs(self.harvester_speed - self.tractor_speed)
        speed_ok = speed_difference <= self.speed_tolerance
        
        if position_ok and speed_ok:
            return True, "Warunki spełnione"
        else:
            return False, f"Pozycja: {position_ok}, Prędkość: {speed_ok}"

    def calculate_pi_control(self, position_error, dt):
        """Oblicza sygnał sterujący regulatora PI."""
        # Człon proporcjonalny
        p_term = self.Kp * position_error
        
        # Człon całkujący
        self.integral_error += position_error * dt
        i_term = self.Ki * self.integral_error
        
        # Sygnał sterujący
        control_signal = p_term + i_term
        
        # Zwróć szczegółowe informacje o członach
        return {
            'control_signal': control_signal,
            'p_term': p_term,
            'i_term': i_term,
            'integral_error': self.integral_error
        }

    def apply_anti_windup(self, target_speed, actual_speed, dt):
        """Stosuje mechanizm anti-windup."""
        # Sprawdź czy sygnał sterujący jest nasycony
        if target_speed >= self.max_speed:
            # Sygnał nasycony w górę - blokuj integrator jeśli błąd ma ten sam znak
            if self.integral_error > 0:
                self.integral_error -= (target_speed - self.max_speed) * dt / self.Ki
        elif target_speed <= self.min_speed:
            # Sygnał nasycony w dół - blokuj integrator jeśli błąd ma ten sam znak
            if self.integral_error < 0:
                self.integral_error -= (target_speed - self.min_speed) * dt / self.Ki

    def apply_limits(self, target_speed):
        """Stosuje ograniczenia prędkości i przyspieszenia."""
        # Ograniczenia prędkości
        target_speed = max(self.min_speed, min(self.max_speed, target_speed))
        
        # Ograniczenia przyspieszenia
        current_time = time.time()
        dt = current_time - self.last_control_time
        if dt > 0:
            max_speed_change = self.max_acceleration * dt
            speed_change = target_speed - self.last_target_speed
            
            if abs(speed_change) > max_speed_change:
                if speed_change > 0:
                    target_speed = self.last_target_speed + max_speed_change
                else:
                    target_speed = self.last_target_speed - max_speed_change
        
        self.last_target_speed = target_speed
        self.last_control_time = current_time
        
        return target_speed

    def calculate_feedforward(self):
        """Oblicza kompensację prędkości sieczkarni (feed-forward)."""
        return self.harvester_speed

    def control_loop(self):
        """Główna pętla regulacji pozycji."""
        if not self.is_enabled:
            return
        
        # Sprawdź warunki aktywacji tylko przy pierwszym uruchomieniu
        if not self.activation_conditions_checked:
            conditions_ok, condition_msg = self.check_activation_conditions()
            if not conditions_ok:
                self.get_logger().warn(f"Autopilot nieaktywny: {condition_msg}")
                self.publish_status("NIEAKTYWNY", condition_msg)
                return
            else:
                self.get_logger().info(f"Warunki aktywacji spełnione: {condition_msg}")
                self.activation_conditions_checked = True
                # Log nastaw regulatora przy aktywacji
                self.get_logger().info(
                    f"NASTAWY REGULATORA POZYCJI: "
                    f"Kp={self.Kp:.3f}, Ki={self.Ki:.3f}, "
                    f"Tolerancja pozycji={self.position_tolerance:.2f}m, "
                    f"Tolerancja prędkości={self.speed_tolerance:.2f}m/s, "
                    f"Min prędkość={self.min_speed:.2f}m/s, "
                    f"Max prędkość={self.max_speed:.2f}m/s"
                )
        
        # Oblicz błąd pozycji
        position_error = self.target_distance - self.current_distance_longitudinal
        
        # Oblicz dt dla regulatora PI
        current_time = time.time()
        dt = current_time - self.last_control_time
        if dt <= 0:
            dt = 1.0 / self.control_frequency  # Fallback
        
        # Oblicz sygnał sterujący PI
        pi_result = self.calculate_pi_control(position_error, dt)
        speed_correction = pi_result['control_signal']
        
        # Oblicz docelową prędkość z feed-forward
        target_speed = self.calculate_feedforward() + speed_correction
        
        # Zastosuj ograniczenia
        target_speed = self.apply_limits(target_speed)
        
        # Zastosuj anti-windup na końcowej prędkości
        self.apply_anti_windup(target_speed, self.tractor_speed, dt)
        
        # Aktualizuj czas ostatniej regulacji
        self.last_control_time = current_time
        
        # Publikuj docelową prędkość
        speed_msg = Float64()
        speed_msg.data = target_speed
        self.target_speed_publisher.publish(speed_msg)
        
        # Publikuj status z szczegółowymi danymi regulacji
        detailed_status = (
            f"AKTYWNY: Błąd={position_error:.3f}m, "
            f"P={pi_result['p_term']:.3f}m/s, "
            f"I={pi_result['i_term']:.3f}m/s, "
            f"Integral={pi_result['integral_error']:.3f}, "
            f"Korekta={speed_correction:.3f}m/s, "
            f"Cel={target_speed:.3f}m/s"
        )
        self.publish_status("AKTYWNY", detailed_status)

    def publish_status(self, status, message):
        """Publikuje status autopilota."""
        status_msg = String()
        status_msg.data = f"{status}: {message}"
        self.status_publisher.publish(status_msg)

    def publish_health(self):
        """Publikuje status zdrowia węzła."""
        try:
            current_time = time.time()
            
            # Sprawdź timeouty
            distance_timeout = current_time - self.last_distance_update > self.gps_timeout
            tractor_timeout = current_time - self.last_tractor_gps_update > self.gps_timeout
            harvester_timeout = current_time - self.last_harvester_gps_update > self.gps_timeout
            
            # Przygotuj dane health
            health_data = {
                'node_name': 'position_controller_node',
                'is_enabled': self.is_enabled,
                'distance_timeout': distance_timeout,
                'tractor_timeout': tractor_timeout,
                'harvester_timeout': harvester_timeout,
                'current_distance': self.current_distance_longitudinal,
                'tractor_speed': self.tractor_speed,
                'harvester_speed': self.harvester_speed,
                'integral_error': self.integral_error
            }
            
            # Publikuj health
            health_msg = String()
            health_msg.data = str(health_data)
            self.health_publisher.publish(health_msg)
            
        except Exception as e:
            self.get_logger().error(f"Błąd publikacji health: {e}")

def main():
    rclpy.init()
    node = PositionControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
