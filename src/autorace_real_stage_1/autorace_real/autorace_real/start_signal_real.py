#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class StartSignalReal(Node):
    def __init__(self):
        super().__init__("start_signal_real")

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laserscan_callback, qos_profile)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.laser_data = None
        self.state = "waiting"
        self.obstacle_detected = False

        self.get_logger().info("Node started. Place obstacle in front and remove to start.")

    def laserscan_callback(self, msg):
        self.laser_data = [
            r if r > 0.0 and not math.isinf(r) and not math.isnan(r) else 3.5
            for r in msg.ranges
        ]

    def control_loop(self):
        if not self.laser_data:
            return

        twist = Twist()

        front_indices = [(i % len(self.laser_data)) for i in range(345, 360)]
        front_vals = [self.laser_data[i] for i in front_indices if self.laser_data[i] < 3.5]
        front = min(front_vals) if front_vals else 3.5

        # STATE 1: WAITING
        if self.state == "waiting":
            twist.linear.x  = 0.0
            twist.angular.z = 0.0

            if front < 0.5:
                self.obstacle_detected = True
                self.get_logger().info(f"Obstacle detected at {front:.2f}m. Waiting for removal...")

            if self.obstacle_detected and front > 0.5:
                self.state = "driving"
                self.get_logger().info("Obstacle removed! Starting to drive...")

        # STATE 2: DRIVING
        elif self.state == "driving":
            if front > 0.4:
                twist.linear.x  = 0.2
                twist.angular.z = 0.0
                self.get_logger().info(f"Driving to wall... front: {front:.2f}m")
            else:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
                self.state = "stopped"
                self.get_logger().info("Wall reached! Stopping.")

        # STATE 3: STOPPED
        elif self.state == "stopped":
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Task complete!")

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = StartSignalReal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
