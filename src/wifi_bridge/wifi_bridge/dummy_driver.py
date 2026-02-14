#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time

class DummyDriver(Node):
    def __init__(self):
        super().__init__('dummy_driver')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.velocity = 0.2  # m/s
        self.turn_rate = 0.2 # rad/s
        
        # Timer (50Hz)
        self.create_timer(0.02, self.update)
        self.get_logger().info("Dummy Driver Started. Publishing /odom and TF for testing.")

    def update(self):
        now = self.get_clock().now()
        dt = 0.02
        
        # Simulate circular motion
        self.x += self.velocity * math.cos(self.th) * dt
        self.y += self.velocity * math.sin(self.th) * dt
        self.th += self.turn_rate * dt
        
        # QUATERNION
        q = [math.cos(self.th/2), 0.0, 0.0, math.sin(self.th/2)] # w, x, y, z
        
        # 1. PUBLISH ODOM MSG
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        self.odom_pub.publish(odom)
        
        # 2. PUBLISH TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        self.tf_broadcaster.sendTransform(t)
        
        # 3. PUBLISH JOINT STATES (Spin wheels)
        # Just fake rotation
        wheel_rot = (time.time() * 5.0) % (2*math.pi)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["front_left_joint", "front_right_joint", "rear_left_joint", "rear_right_joint"]
        js.position = [wheel_rot, wheel_rot, wheel_rot, wheel_rot]
        self.joint_pub.publish(js)

def main():
    rclpy.init()
    node = DummyDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
