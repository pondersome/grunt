import math
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
import rclpy
from rclpy.node import Node

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            'bno055/imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract quaternion from the IMU message
        q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        roll, pitch, yaw = euler_from_quaternion(q)
        # Convert yaw from radians to degrees for readability
        heading = math.degrees(yaw)
        self.get_logger().info(f"Heading (yaw): {heading:.2f} degrees")

def main(args=None):
    rclpy.init(args=args)
    imu_listener = ImuListener()
    rclpy.spin(imu_listener)
    imu_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()