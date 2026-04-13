#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class FollowWall(Node):
    def __init__(self):
        super().__init__("follow_wall_node")

        self.turtlebot_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.turtlebot_subscriber_ = self.create_subscription(
            LaserScan, '/scan', self.laserscan_callback, qos_profile)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ follow_wall_node initialized.")

        self.laser_data = None
        self.state = "find_wall"
        self.wall_follow_distance = 0.5   # Task requires 0.5 m

    def laserscan_callback(self, msg):
        self.laser_data = [
            r if r > 0.0 and not math.isinf(r) and not math.isnan(r) else 3.5
            for r in msg.ranges
        ]

    def get_valid_range(self, index):
        if self.laser_data is None or len(self.laser_data) == 0:
            return 3.5
        return self.laser_data[index % len(self.laser_data)]

    def get_front(self):
        """Min distance in ±30° cone ahead."""
        if self.laser_data is None:
            return 3.5
        n = len(self.laser_data)
        indices = list(range(330, 360)) + list(range(0, 31))
        vals = [self.laser_data[i % n] for i in indices if self.laser_data[i % n] < 3.5]
        return min(vals) if vals else 3.5

    def control_loop(self):
        if not self.laser_data:
            return

        twist = Twist()

        try:
            front       = self.get_front()
            right       = self.get_valid_range(270)  # directly right
            right_front = self.get_valid_range(315)  # front-right diagonal (45° from front)
            right_back  = self.get_valid_range(225)  # back-right diagonal  (45° from back)

        except Exception as e:
            self.get_logger().warn(f"Laser error: {e}")
            return

        # ── STATE MACHINE ──────────────────────────────────────────────────────────
        #
        # CCW rotation → wall always on robot's RIGHT
        # → turn LEFT (angular.z > 0) at every corner
        #
        # Sensor indices (standard ROS LaserScan, angles increase CCW):
        #   0°  = front    90° = left    180° = back    270° = right

        if self.state == "find_wall":
            if front < 0.6:
                # Wall directly ahead → rotate to put it on our right
                twist.linear.x = 0.0
                self.state = "corner_turn"
                self.get_logger().info("🧱 Wall ahead. Turning left to align.")
            elif right < self.wall_follow_distance + 0.3:
                # Right wall in range → start following
                self.state = "follow_wall"
                self.get_logger().info("➡️ Right wall found. Beginning wall follow.")
            else:
                twist.linear.x = 0.15
                self.get_logger().info(
                    f"🔍 Searching for wall... Front:{front:.2f} Right:{right:.2f}")

        elif self.state == "corner_turn":
            # Turn LEFT (CCW) until:
            #   (a) front is clear  AND
            #   (b) right sensor sees the wall we just turned away from
            if front > 0.6 and right < 0.85:
                twist.angular.z = 0.0
                self.state = "follow_wall"
                self.get_logger().info("✅ Corner complete. Resuming wall follow.")
            else:
                twist.angular.z = 0.4   # CCW = left turn ← correct for CCW traversal
                twist.linear.x  = 0.0
                self.get_logger().info(
                    f"↩️ Turning CCW... Front:{front:.2f} Right:{right:.2f}")

        elif self.state == "follow_wall":
            if front < 0.55:
                # New wall ahead → corner
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
                self.state = "corner_turn"
                self.get_logger().info("🔄 Corner detected → turning left (CCW).")

            elif right > 1.2:
                # Lost right wall (gap / opening)
                twist.linear.x  = 0.1
                twist.angular.z = -0.25   # gentle right curve to reacquire
                self.get_logger().info(f"🔍 Lost right wall. Right:{right:.2f}")

            else:
                # PD controller — maintain right wall at wall_follow_distance
                #
                # error > 0  →  too far from wall  →  turn right (angular.z < 0)
                # error < 0  →  too close to wall  →  turn left  (angular.z > 0)
                #
                # D term via diagonal sensors:
                #   right_front > right_back  →  nose diverging from wall  →  extra right turn
                #   right_front < right_back  →  nose converging on wall   →  ease off right
                error  = right - self.wall_follow_distance
                d_term = right_front - right_back

                kp = 1.0
                kd = 0.4
                correction = kp * error + kd * d_term   # positive → turn right

                correction = max(-0.5, min(0.5, correction))

                twist.linear.x  = 0.18
                twist.angular.z = -correction   # negate: positive error → right turn
                self.get_logger().info(
                    f"🟢 Follow | R:{right:.2f} RF:{right_front:.2f} RB:{right_back:.2f} "
                    f"err:{error:.2f} corr:{correction:.2f}")

        self.turtlebot_publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()