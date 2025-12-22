#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        self.map = None
        self.goal = None

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        self.get_logger().info("A* Global Planner started")

    def map_callback(self, msg):
        self.map = msg

    def goal_callback(self, msg):
        self.goal = msg
        if self.map is not None:
            self.plan_path()

    def plan_path(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info("Planning global path (stub A*)")
        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
