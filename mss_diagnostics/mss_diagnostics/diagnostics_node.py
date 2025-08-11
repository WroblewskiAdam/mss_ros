import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Importuj wszystkie potrzebne wiadomości
from std_msgs.msg import Float64
from my_robot_interfaces.msg import GpsRtk, Gear, DistanceMetrics, StampedInt32, DiagnosticData

# Wartość, która będzie wstawiana w miejsce niedostępnych danych
PLACEHOLDER_INT = 99999
PLACEHOLDER_FLOAT = 99999.0

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')

        # --- Parametry ---
        self.declare_parameter('publish_frequency_hz', 10.0)
        self.declare_parameter('data_timeout_sec', 1.5)

        publish_frequency = self.get_parameter('publish_frequency_hz').get_parameter_value().double_value
        self.data_timeout = self.get_parameter('data_timeout_sec').get_parameter_value().double_value

        # --- Zmienne do przechowywania ostatnich wiadomości ---
        self.last_tractor_gps = None
        self.last_chopper_gps = None
        self.last_servo_pos = None
        self.last_gear = None
        self.last_target_speed = None
        self.last_relative_pos = None

        # --- Jakość usług (QoS) ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Publisher dla zagregowanych danych ---
        self.diag_publisher = self.create_publisher(DiagnosticData, '/diagnostics', 10)

        # --- Indywidualni subskrybenci ---
        # Każdy ma swój własny callback, który tylko zapisuje ostatnią wiadomość
        self.create_subscription(GpsRtk, '/gps_rtk_data_filtered', self.tractor_gps_callback, qos_profile)
        self.create_subscription(GpsRtk, '/gps_rtk_data/chopper', self.chopper_gps_callback, qos_profile)
        self.create_subscription(StampedInt32, '/servo/position', self.servo_pos_callback, qos_profile)
        self.create_subscription(Gear, '/gears', self.gear_callback, qos_profile)
        self.create_subscription(Float64, '/target_speed', self.target_speed_callback, qos_profile)
        self.create_subscription(DistanceMetrics, '/distance_metrics', self.relative_pos_callback, qos_profile)

        # --- Główna pętla (Timer) ---
        # Ta funkcja będzie teraz sercem węzła
        self.timer = self.create_timer(1.0 / publish_frequency, self.main_loop_callback)

        self.get_logger().info(f'Odporny węzeł diagnostyczny uruchomiony. Publikuje z f={publish_frequency} Hz.')
        self.get_logger().info(f'Timeout danych ustawiony na {self.data_timeout}s.')

    # --- Definicje callbacków (tylko zapisują dane) ---
    def tractor_gps_callback(self, msg): self.last_tractor_gps = msg
    def chopper_gps_callback(self, msg): self.last_chopper_gps = msg
    def servo_pos_callback(self, msg): self.last_servo_pos = msg
    def gear_callback(self, msg): self.last_gear = msg
    def target_speed_callback(self, msg): self.last_target_speed = msg
    def relative_pos_callback(self, msg): self.last_relative_pos = msg

    def is_data_fresh(self, msg):
        """Sprawdza, czy wiadomość nie jest zbyt stara."""
        if msg is None:
            return False
        
        # Pobierz czas obecny i czas wiadomości
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        msg_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # Sprawdź różnicę
        return (current_time_sec - msg_time_sec) < self.data_timeout

    def main_loop_callback(self):
        """Główna pętla, która buduje i publikuje wiadomość diagnostyczną."""
        diag_msg = DiagnosticData()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        diag_msg.header.frame_id = "diagnostic_frame"

        # 1. Dane z ciągnika (filtrowane)
        if self.is_data_fresh(self.last_tractor_gps):
            diag_msg.tractor_gps_filtered = self.last_tractor_gps
        else:
            # Tworzymy pustą wiadomość i wypełniamy wartościami domyślnymi
            diag_msg.tractor_gps_filtered.rtk_status = PLACEHOLDER_INT
            diag_msg.tractor_gps_filtered.speed_mps = PLACEHOLDER_FLOAT
            diag_msg.tractor_gps_filtered.heading_deg = PLACEHOLDER_FLOAT

        # 2. Dane z sieczkarni i status BT
        if self.is_data_fresh(self.last_chopper_gps):
            diag_msg.chopper_gps = self.last_chopper_gps
            diag_msg.bt_status = True
        else:
            diag_msg.bt_status = False
            diag_msg.chopper_gps.rtk_status = PLACEHOLDER_INT
            diag_msg.chopper_gps.speed_mps = PLACEHOLDER_FLOAT
            diag_msg.chopper_gps.heading_deg = PLACEHOLDER_FLOAT

        # 3. Dane od serwa
        if self.is_data_fresh(self.last_servo_pos):
            diag_msg.servo_position = self.last_servo_pos
        else:
            diag_msg.servo_position.data = PLACEHOLDER_INT

        # 4. Dane o biegach
        if self.last_gear is not None: # Dla biegów nie sprawdzamy czasu, bo mogą się nie zmieniać
            diag_msg.tractor_gear = self.last_gear
        else:
            diag_msg.tractor_gear.gear = PLACEHOLDER_INT
            diag_msg.tractor_gear.clutch_state = PLACEHOLDER_INT
            
        # 5. Dane z systemu sterowania
        if self.is_data_fresh(self.last_target_speed):
            diag_msg.target_speed = self.last_target_speed
        else:
            diag_msg.target_speed.data = PLACEHOLDER_FLOAT

        if self.last_relative_pos is not None: # Pozycja względna może być publikowana rzadziej
            diag_msg.relative_position = self.last_relative_pos
        else:
            diag_msg.relative_position.distance_straight = PLACEHOLDER_FLOAT
            diag_msg.relative_position.distance_longitudinal = PLACEHOLDER_FLOAT
            diag_msg.relative_position.distance_lateral = PLACEHOLDER_FLOAT

        self.diag_publisher.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()