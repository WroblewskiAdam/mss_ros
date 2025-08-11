import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from my_robot_interfaces.msg import Gear, StampedInt32
from std_msgs.msg import Float64 # <-- NOWY IMPORT

class GearManagerNode(Node):
    """
    Węzeł do automatycznego zarządzania zmianą półbiegów (powershift).
    Decyzje o zmianie podejmowane są na podstawie prędkości zadanej
    i znanych charakterystyk prędkości dla każdego półbiegu.
    """
    def __init__(self):
        super().__init__('gear_manager_node')

        # --- Parametry decyzyjne oparte na charakterystyce prędkości ---
        self.declare_parameter('powershift_max_speeds', [2.9, 3.6, 4.1, 5.6], 
                               description="Lista maksymalnych prędkości [m/s] dla półbiegów 1, 2, 3, 4.")
        self.declare_parameter('upshift_threshold_percent', 0.95,
                               description="Procent maksymalnej prędkości biegu, po przekroczeniu którego nastąpi zmiana w górę.")
        self.declare_parameter('downshift_threshold_percent', 0.85,
                               description="Procent maksymalnej prędkości NIŻSZEGO biegu, poniżej którego nastąpi redukcja.")
        self.declare_parameter('shift_cooldown_sec', 4.0)
        self.declare_parameter('initial_powershift', 1)
        self.declare_parameter('max_powershift', 4)

        self.powershift_max_speeds = self.get_parameter('powershift_max_speeds').get_parameter_value().double_array_value
        self.upshift_thresh_percent = self.get_parameter('upshift_threshold_percent').get_parameter_value().double_value
        self.downshift_thresh_percent = self.get_parameter('downshift_threshold_percent').get_parameter_value().double_value
        self.shift_cooldown = self.get_parameter('shift_cooldown_sec').get_parameter_value().double_value
        self.initial_powershift = self.get_parameter('initial_powershift').get_parameter_value().integer_value
        self.max_powershift = self.get_parameter('max_powershift').get_parameter_value().integer_value

        # --- Zmienne stanu ---
        self.target_speed = 0.0
        self.current_mechanical_gear = 0
        self.clutch_pressed = True
        self.current_powershift = self.initial_powershift
        self.last_shift_time = self.get_clock().now()

        # --- Subskrypcje ---
        self.create_subscription(Gear, '/gears', self.gear_callback, 10)
        self.create_subscription(Float64, '/target_speed', self.target_speed_callback, 10) # <-- NOWA SUBSKRYPCJA

        # --- Klienci usług do zmiany biegów ---
        self.shift_up_client = self.create_client(SetBool, 'gear_shift_up')
        self.shift_down_client = self.create_client(SetBool, 'gear_shift_down')
        while not self.shift_up_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Oczekiwanie na serwis /gear_shift_up...')
        while not self.shift_down_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Oczekiwanie na serwis /gear_shift_down...')

        # --- Główna pętla decyzyjna ---
        self.decision_timer = self.create_timer(0.5, self.decision_loop) # 2 Hz

        self.get_logger().info("Menedżer półbiegów uruchomiony (logika oparta na profilu prędkości).")
        self.get_logger().info(f"Maksymalne prędkości półbiegów: {self.powershift_max_speeds}")

    def gear_callback(self, msg):
        """Odbiera stan biegu głównego i sprzęgła."""
        if self.current_mechanical_gear != msg.gear and msg.gear != 0:
            self.get_logger().info(f"Wykryto zmianę biegu głównego na {msg.gear}. Resetuję półbieg do {self.initial_powershift}.")
            self.current_powershift = self.initial_powershift
        self.current_mechanical_gear = msg.gear
        self.clutch_pressed = (msg.clutch_state == 1)
        
    def target_speed_callback(self, msg):
        """Odbiera aktualną prędkość zadaną."""
        self.target_speed = msg.data

    async def call_shift_service(self, client, direction):
        """Asynchronicznie wywołuje usługę zmiany biegu."""
        if not client.service_is_ready():
            self.get_logger().error(f"Serwis zmiany biegów ({direction}) nie jest dostępny.")
            return False
        
        request = SetBool.Request()
        request.data = True
        future = await client.call_async(request)
        
        if future and future.success:
            self.get_logger().info(f"Pomyślnie zmieniono półbieg: {direction}")
            self.last_shift_time = self.get_clock().now()
            return True
        else:
            self.get_logger().error(f"Nie udało się zmienić półbiegu: {direction}")
            return False

    async def decision_loop(self):
        """Główna pętla, która podejmuje decyzje o zmianie półbiegów."""
        # --- Warunki bezpieczeństwa ---
        if self.clutch_pressed or self.current_mechanical_gear == 0 or self.target_speed == 0.0:
            return

        # --- Cooldown po ostatniej zmianie ---
        time_since_last_shift = (self.get_clock().now() - self.last_shift_time).nanoseconds / 1e9
        if time_since_last_shift < self.shift_cooldown:
            return

        current_gear_index = self.current_powershift - 1

        # --- LOGIKA ZMIANY BIEGU W GÓRĘ ---
        if self.current_powershift < self.max_powershift:
            current_gear_max_speed = self.powershift_max_speeds[current_gear_index]
            upshift_speed_trigger = current_gear_max_speed * self.upshift_thresh_percent
            
            if self.target_speed > upshift_speed_trigger:
                self.get_logger().info(f"UPSHIFT: Cel ({self.target_speed:.2f} m/s) > Próg ({upshift_speed_trigger:.2f} m/s) dla biegu {self.current_powershift}.")
                if await self.call_shift_service(self.shift_up_client, "GÓRA"):
                    self.current_powershift += 1
                return

        # --- LOGIKA ZMIANY BIEGU W DÓŁ ---
        if self.current_powershift > 1:
            lower_gear_index = current_gear_index - 1
            lower_gear_max_speed = self.powershift_max_speeds[lower_gear_index]
            downshift_speed_trigger = lower_gear_max_speed * self.downshift_thresh_percent

            if self.target_speed < downshift_speed_trigger:
                self.get_logger().info(f"DOWNSHIFT: Cel ({self.target_speed:.2f} m/s) < Próg ({downshift_speed_trigger:.2f} m/s) dla biegu {self.current_powershift - 1}.")
                if await self.call_shift_service(self.shift_down_client, "DÓŁ"):
                    self.current_powershift -= 1
                return

def main(args=None):
    rclpy.init(args=args)
    node = GearManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
