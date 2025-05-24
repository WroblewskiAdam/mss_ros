import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, Quaternion
import board
import busio
import time

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno = BNO08X_I2C(i2c)
        time.sleep(1)

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 100)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 100)

        self.timer = self.create_timer(0.01, self.publish_data)  # 10 Hz

    def publish_data(self):
        imu_msg = Imu()
        mag_msg = MagneticField()

        accel = self.bno.acceleration
        gyro = self.bno.gyro
        quat = self.bno.quaternion
        mag = self.bno.magnetic

        if None in (accel, gyro, quat, mag):
            self.get_logger().warn('Brak danych z IMU')
            return

        imu_msg.linear_acceleration = Vector3(x=accel[0], y=accel[1], z=accel[2])
        imu_msg.angular_velocity = Vector3(x=gyro[0], y=gyro[1], z=gyro[2])
        imu_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        mag_msg.magnetic_field.x = mag[0]
        mag_msg.magnetic_field.y = mag[1]
        mag_msg.magnetic_field.z = mag[2]
        mag_msg.header = imu_msg.header

        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
