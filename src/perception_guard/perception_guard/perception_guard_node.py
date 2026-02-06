#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import socket
import json
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 9000

BOULDER_TIMEOUT = 1.0
CRATER_TIMEOUT = 2.0


class PerceptionGuard(Node):

    def __init__(self):
        super().__init__('perception_guard')

        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)

        # ROS Interfaces (JAZZY SAFE)
        self.cmd_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_raw',
            self.cmd_cb,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # State
        self.latest_cmd = TwistStamped()
        self.last_boulder_time = 0.0
        self.last_crater_time = 0.0

        # Timers
        self.create_timer(0.02, self.read_udp)
        self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Perception Guard (Jazzy Safe) Running")

    # ---------------- CMD CALLBACK ----------------
    def cmd_cb(self, msg):
        self.latest_cmd = msg

    # ---------------- UDP READER ----------------
    def read_udp(self):
        try:
            data, _ = self.sock.recvfrom(4096)
            msg = json.loads(data.decode())

            now = time.time()

            # -------- QR SUPPORT --------
            qr = msg.get("qr", None)
            if qr:
                qr_id = qr.get("id", "NONE")
                qr_x = qr.get("x", 0)

                self.get_logger().info(
                    f"QR Detected → ID:{qr_id}  X:{qr_x}"
                )

            # -------- EVENTS SUPPORT --------
            # Use .get() so it doesn't crash if 'events' is missing
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
            self.get_logger().warn(f"UDP Parse Error: {e}")

    # ---------------- CONTROL LOOP ----------------
    def control_loop(self):
        # SAFETY CHECK: Don't publish if we haven't received a command yet
        if not self.latest_cmd.header.frame_id:
             return

        now = time.time()

        out = TwistStamped()

        # CRITICAL → Copy header (Time Sync)
        out.header = self.latest_cmd.header

        # Copy original velocities
        out.twist.linear.x = self.latest_cmd.twist.linear.x
        out.twist.angular.z = self.latest_cmd.twist.angular.z

        # -------- SAFETY LOGIC --------
        if now - self.last_crater_time < CRATER_TIMEOUT:
            out.twist.linear.x = 0.0
            out.twist.angular.z = 0.0

        elif now - self.last_boulder_time < BOULDER_TIMEOUT:
            out.twist.linear.x = min(out.twist.linear.x, 0.2)

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionGuard()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()