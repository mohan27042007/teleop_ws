import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        # Publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Timer to publish placeholder IMU messages at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_placeholder)

        self.get_logger().info("IMU Bridge running (placeholder mode). Waiting for real IMU hardware...")

    def publish_placeholder(self):
        """Publish a zero-filled IMU message so EKF can start without errors."""
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # --- Leave all values zero (placeholder) ---
        # Orientation (quaternion)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # Angular velocity (gyro)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # Linear acceleration
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0

        # Covariances (non-zero so EKF accepts them)
        msg.orientation_covariance = [0.1, 0, 0,
                                      0, 0.1, 0,
                                      0, 0, 0.1]

        msg.angular_velocity_covariance = [0.05, 0, 0,
                                           0, 0.05, 0,
                                           0, 0, 0.05]

        msg.linear_acceleration_covariance = [0.1, 0, 0,
                                              0, 0.1, 0,
                                              0, 0, 0.1]

        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
