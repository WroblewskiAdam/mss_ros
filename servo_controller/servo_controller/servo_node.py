import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from adafruit_servokit import ServoKit
import time
import signal
import sys

# ros2 topic pub /servo_angle std_msgs/msg/Int32 '{data: 90}'
class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Publisher for current servo position
        self.position_publisher = self.create_publisher(Int32, 'servo/position', 10)
        
        # Subscription to target servo angle
        self.subscription = self.create_subscription(
            Int32,
            'servo_angle',
            self.angle_callback,
            10)
        
        self.kit = ServoKit(channels=16)
        self.servo_channel = 0
        self.kit.servo[self.servo_channel].set_pulse_width_range(500, 2500)
        
        self.get_logger().info('Servo controller node started.')

        # Initialize servo to 0 degrees at startup
        self.set_angle(0)

    def angle_callback(self, msg):
        angle = msg.data
        if 0 <= angle <= 180:
            self.get_logger().info(f"Moving servo to {angle} degrees")
            self.smooth_move(self.servo_channel, angle, speed=10)
        else:
            self.get_logger().warn('Received angle out of bounds (0-180). Ignoring.')

    def scale_speed(self, user_speed):
        return int(1 + ((user_speed - 1) * (99 / 9)))

    def smooth_move(self, channel, target_angle, speed):
        current_angle = self.kit.servo[channel].angle
        if current_angle is None:
            current_angle = 0
        else:
            current_angle = int(current_angle)

        scaled_speed = self.scale_speed(speed)
        if scaled_speed >= 100:
            self.kit.servo[channel].angle = target_angle
            self.publish_position(target_angle)
            return

        delay = 0.1 * ((100 - scaled_speed) / 100) ** 2
        step = 1 if target_angle > current_angle else -1

        for angle in range(current_angle, target_angle + step, step):
            self.kit.servo[channel].angle = angle
            self.publish_position(angle)
            time.sleep(delay)

    def publish_position(self, angle):
        msg = Int32()
        msg.data = angle
        self.position_publisher.publish(msg)

    def set_angle(self, angle, publish=True):
        try:
            self.kit.servo[self.servo_channel].angle = angle
            if publish:
                self.publish_position(angle)
        except Exception as e:
            self.get_logger().error(f'Error setting angle: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected! Shutting down...')
    finally:
        # Set servo to 0 without publishing (bo publisher będzie zamykany)
        node.set_angle(0, publish=False)
        time.sleep(0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

