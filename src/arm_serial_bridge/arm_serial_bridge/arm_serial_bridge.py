#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import sys

SERIAL_PORT = "/dev/ttyACM0"   # Change if needed
BAUDRATE = 115200


class ArmSerialBridge(Node):
    def __init__(self):
        super().__init__('arm_serial_bridge')

        # Subscribe to arm command topic
        self.subscription = self.create_subscription(
            String,
            '/arm_command',
            self.cmd_callback,
            10
        )

        self.ser = None
        self.connect_serial()

        self.get_logger().info("arm_serial_bridge started.")

    # Try to open serial port
    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            time.sleep(0.2)  # allow Arduino to auto-reset
            self.get_logger().info(f"Connected to {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None

    # Called whenever /arm_command is published
    def cmd_callback(self, msg: String):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Serial not open — reconnecting...")
            self.connect_serial()
            if self.ser is None:
                return

        cmd = msg.data.strip() + "\n"

        try:
            self.ser.write(cmd.encode('ascii'))
            self.ser.flush()
            self.get_logger().info(f"Sent to Arduino: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")
            self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean shutdown
    if node.ser and node.ser.is_open:
        node.ser.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
