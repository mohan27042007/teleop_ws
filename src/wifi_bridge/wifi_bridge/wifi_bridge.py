import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import socket
import math
import time

# --- CONFIG: set your ESP32 IP/PORT here ---
ESP32_IP = "10.32.13.63"   # <<< REPLACE with your ESP32 IP if different >>>
ESP32_PORT = 8888

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (x, y, z, w)
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class WifiBridge(Node):
    def __init__(self):
        super().__init__('wifi_bridge_node')

        # Parameters
        self.declare_parameter('wheel_base', 0.31348)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('ticks_per_rev', 1000.0) # TODO: Tune this!

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False) # Non-blocking mode

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.create_subscription(String, '/arm_command', self.arm_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.last_time = self.get_clock().now()

        # Timer for reading UDP
        self.create_timer(0.02, self.read_udp_loop) # 50Hz check

        self.get_logger().info(f'WifiBridge started. Target: {ESP32_IP}:{ESP32_PORT}')

    def twist_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # Simple mapping -> integer motor speeds (adjust scale if needed)
        # TODO: Use proper kinematics to convert m/s to PWM if possible, 
        # but for now keeping the simple scaling from previous code.
        left_speed = int((linear - angular) * 200)
        right_speed = int((linear + angular) * 200)

        command = f"M{left_speed}_{right_speed}"
        self.send_udp(command)

    def arm_callback(self, msg: String):
        command = msg.data.strip()
        if not command:
            return
        if command[0] != 'A':
            command = 'A' + command
        self.send_udp(command)

    def send_udp(self, message: str):
        try:
            self.sock.sendto(message.encode('utf-8'), (ESP32_IP, ESP32_PORT))
        except Exception as e:
            self.get_logger().error(f"UDP Send Error: {e}")

    def read_udp_loop(self):
        try:
            while True: # Read all available packets
                data, addr = self.sock.recvfrom(1024)
                message = data.decode('utf-8').strip()
                self.process_packet(message)
        except BlockingIOError:
            pass # No more data
        except Exception as e:
            self.get_logger().error(f"UDP Read Error: {e}")

    def process_packet(self, message):
        # Expected format: E:left_ticks:right_ticks
        if not message.startswith('E:'):
            return

        try:
            parts = message.split(':')
            if len(parts) != 3:
                return
            
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
            
            self.update_odometry(left_ticks, right_ticks)
        except ValueError:
            pass

    def update_odometry(self, left_ticks, right_ticks):
        current_time = self.get_clock().now()
        
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_time = current_time
            return

        # Calculate delta ticks
        d_left_ticks = left_ticks - self.prev_left_ticks
        d_right_ticks = right_ticks - self.prev_right_ticks

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Convert to distance
        # Distance = (ticks / ticks_per_rev) * (2 * pi * radius)
        d_left = (d_left_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        d_right = (d_right_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

        # Differential Drive Kinematics
        d_dist = (d_right + d_left) / 2.0
        d_th = (d_right - d_left) / self.wheel_base

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if d_dist != 0:
            dx = d_dist * math.cos(self.th)
            dy = d_dist * math.sin(self.th)
            self.x += dx
            self.y += dy
        
        self.th += d_th

        # Publish Odometry
        q = euler_to_quaternion(0, 0, self.th)
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocity (Twist)
        if dt > 0:
            vx = d_dist / dt
            vth = d_th / dt
            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

        # --- after setting odom.pose.pose and odom.twist.twist ---

        # -------------------------
        # Add covariance HERE
        # -------------------------
        odom.pose.covariance = [
            0.02, 0, 0, 0, 0, 0,
            0, 0.02, 0, 0, 0, 0,
            0, 0, 99999, 0, 0, 0,
            0, 0, 0, 99999, 0, 0,
            0, 0, 0, 0, 99999, 0,
            0, 0, 0, 0, 0, 0.1
        ]

        odom.twist.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 99999, 0, 0, 0,
            0, 0, 0, 99999, 0, 0,
            0, 0, 0, 0, 99999, 0,
            0, 0, 0, 0, 0, 0.2
        ]

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

if __name__ == '__main__':
    main()
