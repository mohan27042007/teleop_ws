import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket

# --- CONFIG: set your ESP32 IP/PORT here ---
ESP32_IP = "10.32.13.63"   # <<< REPLACE with your ESP32 IP if different >>>
ESP32_PORT = 8888

class WifiBridge(Node):
    def __init__(self):
        super().__init__('wifi_bridge_node')

        # UDP socket for sending packets to ESP32
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscribe to /cmd_vel and /arm_command
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.create_subscription(String, '/arm_command', self.arm_callback, 10)

        self.get_logger().info(f'UDP bridge to ESP32 at {ESP32_IP}:{ESP32_PORT}')

    def twist_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # Simple mapping -> integer motor speeds (adjust scale if needed)
        left_speed = int((linear - angular) * 200)
        right_speed = int((linear + angular) * 200)

        command = f"M{left_speed}_{right_speed}"
        self.send_udp(command)
        # debug log
        self.get_logger().debug(f"Twist -> {command}")

    def arm_callback(self, msg: String):
        # arm_command expected as a string like: A60_120_90
        command = msg.data.strip()
        if not command:
            return
        # accept both formats: numeric or prefixed with A
        if command[0] != 'A':
            command = 'A' + command
        self.send_udp(command)
        self.get_logger().debug(f"Arm -> {command}")

    def send_udp(self, message: str):
        try:
            self.sock.sendto(message.encode('utf-8'), (ESP32_IP, ESP32_PORT))
            self.get_logger().info(f"Sent UDP: {message}")
        except Exception as e:
            self.get_logger().error(f"UDP Error: {e}")

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

