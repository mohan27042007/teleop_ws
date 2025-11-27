#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import math
import time

class MotorNode(Node):
    def __init__(self):
        super().__init__('rover_motor_node')

        # Parameters (tune these for your robot)
        self.declare_parameter('wheel_base', 0.31348)    # meters (L)
        self.declare_parameter('wheel_radius', 0.05)     # meters
        self.declare_parameter('max_linear_mps', 1.0)    # m/s
        self.declare_parameter('max_angular_rps', 2.0)   # rad/s
        self.declare_parameter('max_pwm', 255.0)         # pwm scale (0..max_pwm)
        self.declare_parameter('pid_scale', 1.0)         # placeholder if you add scale
        self.declare_parameter('cmd_timeout', 0.5)       # seconds watchdog
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear = self.get_parameter('max_linear_mps').value
        self.max_angular = self.get_parameter('max_angular_rps').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE

        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, qos)
        self.pub = self.create_publisher(
            Float32MultiArray, 'motor_pwm', qos)

        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)  # 20Hz

        self.get_logger().info('rover_motor_node started: wheel_base=%.3f wheel_r=%.3f' %
                               (self.wheel_base, self.wheel_radius))

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        lin = msg.linear.x
        ang = msg.angular.z

        # clamp
        lin = max(min(lin, self.max_linear), -self.max_linear)
        ang = max(min(ang, self.max_angular), -self.max_angular)

        # differential drive forward kinematics:
        # v_left = v - (omega * L/2)
        # v_right = v + (omega * L/2)
        v_left = lin - (ang * self.wheel_base / 2.0)
        v_right = lin + (ang * self.wheel_base / 2.0)

        # convert linear m/s to angular vel rad/s: w = v / r
        w_left = 0.0 if abs(v_left) < 1e-6 else (v_left / self.wheel_radius)
        w_right = 0.0 if abs(v_right) < 1e-6 else (v_right / self.wheel_radius)

        # convert to a pwm scale (simple linear mapping)
        # Compute maximum wheel angular speed corresponding to max_linear
        max_wheel_speed = self.max_linear / self.wheel_radius
        # norm in [-1, 1]
        left_norm = w_left / max_wheel_speed
        right_norm = w_right / max_wheel_speed

        left_norm = max(min(left_norm, 1.0), -1.0)
        right_norm = max(min(right_norm, 1.0), -1.0)

        left_pwm = left_norm * self.max_pwm
        right_pwm = right_norm * self.max_pwm

        if self.invert_left:
            left_pwm = -left_pwm
        if self.invert_right:
            right_pwm = -right_pwm

        out = Float32MultiArray()
        out.data = [float(left_pwm), float(right_pwm)]
        self.pub.publish(out)

    def watchdog_check(self):
        # if last command older than timeout, stop motors
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            stop_msg = Float32MultiArray()
            stop_msg.data = [0.0, 0.0]
            self.pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
