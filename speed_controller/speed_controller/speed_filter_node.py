import rclpy
from rclpy.node import Node
# from gps_rtk_msgs.msg import GpsRtk
from my_robot_interfaces.msg import GpsRtk
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 
from scipy.signal import butter, lfilter_zi, lfilter
import numpy as np

class SpeedFilterNode(Node):
    def __init__(self):
        super().__init__('speed_filter_node')

        # --- Parametry filtru ---
        self.declare_parameter('filter_cutoff_hz', 0.8)
        self.declare_parameter('filter_order', 2)
        self.declare_parameter('sampling_frequency_hz', 20.0)

        fpass = self.get_parameter('filter_cutoff_hz').get_parameter_value().double_value
        order = self.get_parameter('filter_order').get_parameter_value().integer_value
        fs = self.get_parameter('sampling_frequency_hz').get_parameter_value().double_value

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            GpsRtk,
            '/gps_rtk_data',
            self.listener_callback,
            qos_profile=sensor_qos_profile)

        self.publisher_ = self.create_publisher(GpsRtk, '/gps_rtk_data_filtered', qos_profile=sensor_qos_profile)

        self.b, self.a = butter(order, fpass, btype='low', analog=False, fs=fs)
        self.zi = lfilter_zi(self.b, self.a)

        self.get_logger().info(f"Filtr prędkości uruchomiony.")

    def listener_callback(self, msg):
        self.get_logger().debug(f'>>> Otrzymano dane w callbacku! Prędkość surowa: {msg.speed_mps:.2f}')
        
        raw_speed = msg.speed_mps
        filtered_speed, self.zi = lfilter(self.b, self.a, [raw_speed], zi=self.zi)

        filtered_msg = msg 
        filtered_msg.speed_mps = filtered_speed[0]

        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()