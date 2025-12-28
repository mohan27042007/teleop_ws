import rcl9py
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json
import time

UDP_IP = "0.0.0.0"     # listen on all interfaces
UDP_PORT = 9000        # MUST match Pi4 sender

BOULDER_TIMEOUT = 1.0  # seconds
CRATER_TIMEOUT  = 2.0

class PerceptionGuard(Node):
    def __init__(self):
        super().__init__('perception_guard')

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)

        # ROS interfaces
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_cb, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # State
        self.last_boulder_time = 0.0
        self.last_crater_time  = 0.0
        self.latest_cmd = Twist()

        # Timers
        self.create_timer(0.02, self.read_udp)     # 50 Hz
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Perception Guard running")

    def cmd_cb(self, msg):
        self.latest_cmd = msg

    def read_udp(self):
    try:
        data, _ = self.sock.recvfrom(2048)
        msg = json.loads(data.decode())

        now = time.time()
        events = msg.get("events", [])

        for event in events:
            etype = event.get("type", "").upper()

            if etype == "BOULDER":
                self.last_boulder_time = now
            elif etype == "CRATER":
                self.last_crater_time = now

    except BlockingIOError:
        pass
    except Exception as e:
        self.get_logger().warn(f"UDP parse error: {e}")

    def control_loop(self):
        now = time.time()
        out = Twist()
        out.linear.x  = self.latest_cmd.linear.x
        out.angular.z = self.latest_cmd.angular.z

        if now - self.last_crater_time < CRATER_TIMEOUT:
            # HARD STOP
            out.linear.x = 0.0
            out.angular.z = 0.0

        elif now - self.last_boulder_time < BOULDER_TIMEOUT:
            # SLOW DOWN
            out.linear.x = min(out.linear.x, 0.2)

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionGuard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
