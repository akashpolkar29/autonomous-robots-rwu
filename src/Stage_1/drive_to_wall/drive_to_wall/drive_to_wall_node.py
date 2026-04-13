#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class DriveToWall(Node): 
    def __init__(self):
        super().__init__("drive_to_wall")

        # Publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        self.get_logger().info("✅ drive_to_wall node started")

    def scan_callback(self, msg):
        twist = Twist()

        # Read front distance
        front_distance = msg.ranges[0]

        # Move or stop
        if front_distance > 1.0:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0.0

        # Publish command
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()