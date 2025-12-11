import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import socket
import math
import time
from sensor_msgs.msg import Imu
from std_msgs.msg import String


ESP32_IP = "10.233.1.63"
ESP32_PORT = 8888


# ----------------------------------------
# Utility: Convert quaternion list to ROS Quaternion
# ----------------------------------------
def make_quaternion(qw, qx, qy, qz):
    q = Quaternion()
    q.w = qw
    q.x = qx
    q.y = qy
    q.z = qz
    return q


class WifiBridge(Node):
    def __init__(self):
        super().__init__('wifi_bridge_node')

        # ---- Robot parameters ----
        self.declare_parameter('wheel_base', 0.31348)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('ticks_per_rev', 1000.0)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        # ---- UDP socket ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0", ESP32_PORT))

        # ---- ROS publishers ----
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)

        # ---- ROS subscribers ----
        self.create_subscription(Twist, "/cmd_vel", self.twist_callback, 10)
        self.create_subscription(String, "/arm_command", self.arm_callback, 10)

        # Odometry state variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_time = self.get_clock().now()

        # IMU state
        self.last_imu_stamp = self.get_clock().now()

        # Read UDP at 50 Hz
        self.create_timer(0.02, self.read_udp_loop)

        self.get_logger().info(f"wifi_bridge started. Listening on {ESP32_PORT}")

    # ----------------------------------------
    # SEND COMMANDS TO ESP32
    # ----------------------------------------
    def twist_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left = int((linear - angular) * 200)
        right = int((linear + angular) * 200)

        command = f"M{left}_{right}"
        self.sock.sendto(command.encode(), (ESP32_IP, ESP32_PORT))

    def arm_callback(self, msg):
        cmd = msg.data.strip()
        if cmd and cmd[0] != 'A':
            cmd = "A" + cmd
        self.sock.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))

    # ----------------------------------------
    # RECEIVE UDP DATA FROM ESP32
    # ----------------------------------------
    def read_udp_loop(self):
        try:
            while True:
                data, _ = self.sock.recvfrom(256)
                message = data.decode().strip()
                self.parse_packet(message)
        except BlockingIOError:
            pass

    # ----------------------------------------
    # PACKET PARSER
    # ----------------------------------------
    def parse_packet(self, msg):

        # ---- Odometry encoder ticks ----
        if msg.startswith("E:"):
            _, l, r = msg.split(":")
            self.update_odometry(int(l), int(r))
            return

        # ---- IMU Packet ----
        if msg.startswith("I:"):
            parts = msg.split(":")
            if len(parts) != 10:
                return

            _, qw, qx, qy, qz, ax, ay, az, gz = parts

            self.publish_imu(
                float(qw), float(qx), float(qy), float(qz),
                float(ax), float(ay), float(az),
                float(gz)
            )
            return

    # ----------------------------------------
    # IMU PUBLISHER
    # ----------------------------------------
    def publish_imu(self, qw, qx, qy, qz, ax, ay, az, gz):

        now = self.get_clock().now()
        msg = Imu()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "imu_link"

        # Orientation (from ESP32 filter)
        msg.orientation = make_quaternion(qw, qx, qy, qz)

        # Gyro (rad/s)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = gz

        # Acceleration (m/s^2)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Covariances (EKF expects non-zero)
        msg.orientation_covariance = [0.1,0,0, 0,0.1,0, 0,0,0.1]
        msg.angular_velocity_covariance = [0.05,0,0, 0,0.05,0, 0,0,0.05]
        msg.linear_acceleration_covariance = [0.2,0,0, 0,0.2,0, 0,0,0.2]

        self.imu_pub.publish(msg)

    # ----------------------------------------
    # ODOMETRY CALCULATION
    # ----------------------------------------
    def update_odometry(self, l_ticks, r_ticks):

        time_now = self.get_clock().now()

        if self.prev_left_ticks is None:
            self.prev_left_ticks = l_ticks
            self.prev_right_ticks = r_ticks
            self.last_time = time_now
            return

        # Tick differences
        dl = l_ticks - self.prev_left_ticks
        dr = r_ticks - self.prev_right_ticks

        self.prev_left_ticks = l_ticks
        self.prev_right_ticks = r_ticks

        # Convert ticks → meters
        dist_l = (dl / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        dist_r = (dr / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

        d_dist = (dist_l + dist_r) / 2.0
        d_th = (dist_r - dist_l) / self.wheel_base

        dt = (time_now - self.last_time).nanoseconds / 1e9
        self.last_time = time_now

        # Integrate
        self.x += d_dist * math.cos(self.th)
        self.y += d_dist * math.sin(self.th)
        self.th += d_th

        # Publish odom
        q = [
            math.cos(self.th/2),
            0.0,
            0.0,
            math.sin(self.th/2)
        ]

        odom = Odometry()
        odom.header.stamp = time_now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])

        odom.twist.twist.linear.x = d_dist / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = d_th / dt if dt > 0 else 0.0

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
