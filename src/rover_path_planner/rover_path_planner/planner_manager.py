#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class PlannerManager(Node):
    def __init__(self):
        super().__init__('planner_manager')
        self.get_logger().info("Planner Manager online")

def main():
    rclpy.init()
    node = PlannerManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
