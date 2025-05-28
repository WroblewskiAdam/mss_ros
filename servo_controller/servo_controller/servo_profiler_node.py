import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import StampedInt32 #
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy #
import time
import signal

class ServoProfilerNode(Node):
    def __init__(self):
        super().__init__('servo_profiler_node')

        self.declare_parameter('start_angle', 0)
        self.declare_parameter('end_angle', 180)
        self.declare_parameter('angle_step', 5)
        self.declare_parameter('hold_duration_sec', 5.0)
        self.declare_parameter('initial_delay_sec', 5.0)

        self.start_angle = self.get_parameter('start_angle').get_parameter_value().integer_value
        self.end_angle = self.get_parameter('end_angle').get_parameter_value().integer_value
        self.angle_step = self.get_parameter('angle_step').get_parameter_value().integer_value
        self.hold_duration = self.get_parameter('hold_duration_sec').get_parameter_value().double_value
        self.initial_delay = self.get_parameter('initial_delay_sec').get_parameter_value().double_value

        self.initialization_failed = False
        if not (0 <= self.start_angle <= 180 and \
                0 <= self.end_angle <= 180 and \
                self.start_angle <= self.end_angle and \
                self.angle_step > 0):
            self.get_logger().error("Nieprawidłowe parametry kątów.")
            self.initialization_failed = True
            return
            
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, #
            history=QoSHistoryPolicy.KEEP_LAST, #
            depth=10 #
        )

        self.publisher_ = self.create_publisher(
            StampedInt32, 
            'servo/set_angle', 
            qos_profile=sensor_qos_profile #
        )
        self.get_logger().info(f"Węzeł profilera serwa uruchomiony. Cel: '{self.publisher_.topic_name}'")
        self.get_logger().info(f"Profilowanie od {self.start_angle} do {self.end_angle} stopni, co {self.angle_step} stopni.")
        self.get_logger().info(f"Czas stabilizacji na każdym kroku: {self.hold_duration} sekund.")

        self.current_target_angle = self.start_angle
        self.profiling_active = False
        self.shutting_down_initiated = False # Flaga do jednorazowego wywołania procedury zamykania

        self.step_timer = None 
        self.start_timer = None 

        self.publish_timer = self.create_timer(1.0 / 20.0, self.publish_current_angle)
        self.start_timer = self.create_timer(self.initial_delay, self.start_profiling_sequence_callback)

    def publish_current_angle(self):
        if self.initialization_failed or self.shutting_down_initiated : # Zmieniona flaga
            return
        
        if self.profiling_active or (not self.profiling_active and self.start_timer is not None):
            msg = StampedInt32()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data = int(self.current_target_angle)
            self.publisher_.publish(msg)

    def start_profiling_sequence_callback(self):
        if self.initialization_failed or self.shutting_down_initiated:
            return
        
        if self.start_timer:
            self.start_timer.cancel()
            self.start_timer = None

        self.get_logger().info(f"Rozpoczynanie sekwencji profilowania po {self.initial_delay}s opóźnienia...")
        self.profiling_active = True
        self.run_next_step()

    def run_next_step(self):
        if self.initialization_failed or not rclpy.ok() or self.shutting_down_initiated:
            return

        if self.current_target_angle > self.end_angle:
            self.get_logger().info("Sekwencja profilowania zakończona.")
            self.profiling_active = False
            self.initiate_shutdown_procedure() # Wywołaj nową metodę
            return

        self.get_logger().info(f"Ustawianie kąta serwa na: {self.current_target_angle} stopni.")
        
        if self.step_timer: 
            self.step_timer.cancel()
            self.step_timer = None

        if rclpy.ok(): 
            self.step_timer = self.create_timer(self.hold_duration, self.schedule_next_angle_update_callback)

    def schedule_next_angle_update_callback(self):
        if self.initialization_failed or not rclpy.ok() or self.shutting_down_initiated:
            return
        
        if self.step_timer:
            self.step_timer.cancel()
            self.step_timer = None

        self.current_target_angle += self.angle_step
        self.run_next_step()
        
    def _set_servo_to_zero_and_wait(self):
        if hasattr(self, 'publisher_') and self.publisher_ is not None and not self.initialization_failed:
            self.get_logger().info("Próba ustawienia serwa na 0 stopni...")
            msg = StampedInt32()
            
            # Jeśli kontekst rclpy wciąż działa, spróbuj pobrać czas
            # W przeciwnym razie, nie ustawiaj stempla (będzie domyślny)
            try:
                if rclpy.ok() and self.executor is not None and not self.executor.shutdown_requested: # Lepsze sprawdzenie
                     msg.header.stamp = self.get_clock().now().to_msg()
                else:
                    self.get_logger().warn("Kontekst rclpy jest zamykany lub nieaktywny, pomijam ustawianie stempla czasowego dla komendy serwa.")
            except Exception as e:
                 self.get_logger().warn(f"Nie można pobrać aktualnego czasu podczas ustawiania serwa na 0: {e}")

            msg.data = 0 
            
            for i in range(10): 
                try:
                    # Sprawdź, czy publisher jest wciąż ważny
                    if self.publisher_.is_valid:
                        self.publisher_.publish(msg)
                        self.get_logger().debug(f"Wysłano komendę serwo=0 (próba {i+1}/10)")
                    else:
                        self.get_logger().warn("Publisher nie jest już ważny, przerywam wysyłanie komendy do serwa.")
                        break
                except rclpy.exceptions.RCLError as rcle:
                    self.get_logger().error(f"RCLError podczas publikowania komendy serwo=0: {rcle}. Kontekst mógł zostać zamknięty.")
                    break
                except Exception as e:
                    self.get_logger().error(f"Inny błąd podczas publikowania komendy serwo=0: {e}")
                    break 
                time.sleep(0.05) 
            self.get_logger().info("Zakończono próbę ustawienia serwa na 0.")

    def initiate_shutdown_procedure(self):
        """Rozpoczyna procedurę zamykania węzła."""
        if self.shutting_down_initiated: # Zapobiegaj wielokrotnemu wywołaniu
            return
        self.shutting_down_initiated = True
        self.get_logger().info("Inicjowanie procedury zamykania węzła ServoProfilerNode...")

        # Anuluj wszystkie aktywne timery
        if hasattr(self, 'publish_timer') and self.publish_timer:
            self.publish_timer.cancel()
            self.publish_timer = None
        if hasattr(self, 'start_timer') and self.start_timer:
            self.start_timer.cancel()
            self.start_timer = None
        if hasattr(self, 'step_timer') and self.step_timer:
            self.step_timer.cancel()
            self.step_timer = None
        
        self._set_servo_to_zero_and_wait()
        
        # Dopiero teraz, po wykonaniu operacji specyficznych dla węzła, zamknij rclpy
        if rclpy.ok():
            self.get_logger().info("Inicjowanie rclpy.shutdown() z procedury zamykania węzła.")
            rclpy.shutdown()

    def destroy_node(self):
        # Metoda destroy_node jest automatycznie wywoływana przez rclpy po shutdown,
        # ale nasza główna logika czyszczenia jest teraz w initiate_shutdown_procedure.
        # Możemy tu zostawić tylko super().destroy_node() lub dodatkowe logowanie.
        self.get_logger().info("Wywołano destroy_node dla ServoProfilerNode (po rclpy.shutdown).")
        super().destroy_node()


# Globalna referencja do węzła, aby handler sygnału mógł go dosięgnąć
_profiler_node_instance = None

def signal_handler(sig, frame):
    global _profiler_node_instance
    print('Otrzymano Ctrl+C (SIGINT)! Inicjuję zamykanie...')
    if _profiler_node_instance is not None:
        _profiler_node_instance.initiate_shutdown_procedure()
    else:
        # Jeśli węzeł nie został jeszcze utworzony, po prostu zamknij rclpy
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    global _profiler_node_instance
    rclpy.init(args=args)
    
    initialization_failed_in_constructor = False
    
    # Rejestracja handlera sygnału PRZED utworzeniem węzła i spin()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        _profiler_node_instance = ServoProfilerNode()
        if hasattr(_profiler_node_instance, 'initialization_failed') and _profiler_node_instance.initialization_failed:
            initialization_failed_in_constructor = True
            _profiler_node_instance.get_logger().error("Inicjalizacja węzła nie powiodła się. Zamykanie.")
        elif rclpy.ok():
             rclpy.spin(_profiler_node_instance) 
    
    except (KeyboardInterrupt, rclpy.exceptions.RCLError) as e: 
        logger = rclpy.logging.get_logger('servo_profiler_main')
        logger.info(f"Przerwano spin() lub napotkano RCLError (oczekiwane przy zamykaniu przez Ctrl+C): {e}")
    except Exception as e:
        logger = rclpy.logging.get_logger('servo_profiler_main')
        logger.error(f"Wystąpił nieoczekiwany błąd: {e}", exc_info=True)
    finally:
        logger = rclpy.logging.get_logger('servo_profiler_main_finally')
        logger.info("Blok finally osiągnięty.")
        
        # Procedura zamykania powinna być już zainicjowana przez initiate_shutdown_procedure()
        # lub przez błąd. Tutaj tylko upewniamy się, że rclpy jest zamknięte.
        if _profiler_node_instance and not initialization_failed_in_constructor:
             if not _profiler_node_instance.shutting_down_initiated:
                 # Jeśli z jakiegoś powodu shutdown nie został zainicjowany (np. błąd przed spin)
                 logger.warn("Procedura zamykania nie została zainicjowana, próbuję teraz...")
                 _profiler_node_instance.initiate_shutdown_procedure() # To wywoła rclpy.shutdown()
             # destroy_node jest wywoływane automatycznie po rclpy.shutdown() jeśli węzeł jest częścią wykonawcy
        
        # Upewnij się, że rclpy jest zamknięte, jeśli nie zostało jeszcze zamknięte
        # To może być nadmiarowe, jeśli initiate_shutdown_procedure() zawsze działa
        if rclpy.ok():
            logger.info("Zamykanie rclpy z końca bloku finally (jeśli jeszcze aktywne).")
            rclpy.shutdown()
        logger.info("Zakończono main.")
        _profiler_node_instance = None # Wyczyść globalną referencję

if __name__ == '__main__':
    main()