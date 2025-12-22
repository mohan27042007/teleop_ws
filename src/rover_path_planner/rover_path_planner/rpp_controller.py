#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class RPPController(Node):
    def __init__(self):
        super().__init__('rpp_controller')

        self.path = None
        self.odom = None

        self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("RPP Controller started")

    def path_callback(self, msg):
        self.path = msg

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg

    def control_loop(self):
        if self.path is None or self.odom is None:
            return

        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0

        # TODO: reactive path following logic
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RPPController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
