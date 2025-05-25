# w pliku mss_visualization/mss_visualization/imu_visualizer_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
# === NOWE IMPORTY ===
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# ====================

class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        
        # === NOWA SEKCJA: Definicja profilu QoS ===
        # Kopiujemy profil z węzła imu_node, aby zapewnić kompatybilność
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # =========================================
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            # Używamy zdefiniowanego profilu QoS
            qos_profile=sensor_qos_profile
        )
        self.get_logger().info('Węzeł wizualizacji IMU uruchomiony z poprawnym QoS.')

    def imu_callback(self, msg):
        # ... (reszta kodu bez zmian)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    # ... (bez zmian)
    rclpy.init(args=args)
    node = ImuVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()