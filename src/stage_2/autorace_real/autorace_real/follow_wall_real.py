#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class SmartWallFollower(Node):
    def __init__(self):
        super().__init__("smart_wall_follower")

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laserscan_callback, qos_profile)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.laser_data = None

        self.state = "waiting"
        self.obstacle_detected = False

        self.desired_distance = None   # IMPORTANT: learned dynamically

        self.get_logger().info("Smart wall follower started")

    # -------------------------------
    def laserscan_callback(self, msg):
        self.laser_data = [
            r if r > 0.0 and not math.isinf(r) and not math.isnan(r) else 3.5
            for r in msg.ranges
        ]

    def get_range(self, index):
        if not self.laser_data:
            return 3.5
        return self.laser_data[index % len(self.laser_data)]

    def get_front_75(self):
        n = len(self.laser_data)
        indices = list(range(322, n)) + list(range(0, 38))
        vals = [self.laser_data[i] for i in indices if self.laser_data[i] < 3.5]
        return min(vals) if vals else 3.5

    # -------------------------------
    def control_loop(self):
        if not self.laser_data:
            return

        twist = Twist()

        try:
            front = self.get_front_75()
            right = self.get_range(270)
            right_front = self.get_range(290)
            right_back = self.get_range(250)
        except:
            return

        # -------------------------------
        # STATE 1: WAITING
        # -------------------------------
        if self.state == "waiting":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if front < 0.5:
                self.obstacle_detected = True

            if self.obstacle_detected and front > 0.5:
                self.state = "go_to_wall"

        # -------------------------------
        # STATE 2: GO TO WALL
        # -------------------------------
        elif self.state == "go_to_wall":
            if front > 0.5:
                twist.linear.x = 0.18
            else:
                self.state = "turn_left"

        # -------------------------------
        # STATE 3: TURN LEFT
        # -------------------------------
        elif self.state == "turn_left":
            twist.angular.z = 0.4

            if right < 0.6:
                self.desired_distance = right   # STORE DISTANCE HERE
                self.state = "follow_wall"

        # -------------------------------
        # STATE 4: FOLLOW WALL
        # -------------------------------
        elif self.state == "follow_wall":

            # Obstacle ahead → turn left
            if front < 0.4:
                twist.linear.x = 0.0
                twist.angular.z = 0.4

            # Wall lost → strong right turn
            elif right > self.desired_distance + 0.3:
                twist.linear.x = 0.02
                twist.angular.z = -0.6

            else:
                # distance error
                error = self.desired_distance - right

                # angle correction
                angle_error = right_front - right_back
                angle_error = max(min(angle_error, 0.1), -0.1)

                # deadband (reduce zig-zag)
                if abs(error) < 0.02:
                    error = 0.0

                # smooth control
                k_p = 0.5
                k_d = 0.2

                control = k_p * error + k_d * angle_error

                # clamp
                control = max(min(control, 0.3), -0.3)

                # adaptive speed
                if abs(control) > 0.2:
                    speed = 0.08
                else:
                    speed = 0.14

                twist.linear.x = speed
                twist.angular.z = control

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SmartWallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

