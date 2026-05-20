import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from custom_interfaces.action import TravelNoCrashing
from custom_interfaces.msg import DangerZones


class CollisionAvoiderNode(Node):

    def __init__(self):
        super().__init__('collision_avoider_node')

        self.cb_action = MutuallyExclusiveCallbackGroup()
        self.cb_topics = MutuallyExclusiveCallbackGroup()

        # Robot position
        self.current_x = None
        self.current_y = None

        self.start_x = None
        self.start_y = None

        # Danger information
        self.danger_zones = DangerZones()

        self.most_dangerous = 'none'
        self.highest_danger = 0

        # QoS for odom
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=odom_qos,
            callback_group=self.cb_topics
        )

        self.danger_sub = self.create_subscription(
            DangerZones,
            '/danger_zones',
            self.danger_callback,
            10,
            callback_group=self.cb_topics
        )

        # Action server
        self.action_server = ActionServer(
            self,
            TravelNoCrashing,
            '/drive_no_crashing',
            self.execute_callback,
            callback_group=self.cb_action
        )

        self.get_logger().info('CollisionAvoiderNode started')

    # -------------------------------------------------
    # CALLBACKS
    # -------------------------------------------------

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def danger_callback(self, msg):

        self.danger_zones = msg

        zones = {
            'front_left': msg.front_left,
            'front_right': msg.front_right,
            'back_left': msg.back_left,
            'back_right': msg.back_right,
        }

        max_zone = max(zones, key=zones.get)
        max_value = zones[max_zone]

        if max_value > self.highest_danger:
            self.highest_danger = max_value
            self.most_dangerous = max_zone

    # -------------------------------------------------
    # HELPERS
    # -------------------------------------------------

    def get_traveled_distance(self):

        if (
            self.start_x is None or
            self.start_y is None or
            self.current_x is None or
            self.current_y is None
        ):
            return 0.0

        return math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    # -------------------------------------------------
    # ACTION CALLBACK
    # -------------------------------------------------

    def execute_callback(self, goal_handle):

        target_distance = goal_handle.request.target_distance

        self.get_logger().info(
            f'Received goal: {target_distance:.2f} meters'
        )

        # Wait for odometry
        start_wait = time.time()

        while self.current_x is None:

            if time.time() - start_wait > 10.0:
                self.get_logger().error('No odom received')

                goal_handle.abort()

                result = TravelNoCrashing.Result()
                return result

            time.sleep(0.1)

        # Reset tracking
        self.start_x = self.current_x
        self.start_y = self.current_y

        self.highest_danger = 0
        self.most_dangerous = 'none'

        # Timing
        start_time = time.time()
        last_feedback_time = start_time
        last_log_time = start_time

        feedback_msg = TravelNoCrashing.Feedback()

        # Main control loop
        while rclpy.ok():

            # Handle cancel request
            if goal_handle.is_cancel_requested:

                self.stop_robot()

                goal_handle.canceled()

                self.get_logger().info('Goal canceled')

                result = TravelNoCrashing.Result()
                return result

            # Safety timeout
            if time.time() - start_time > 40.0:

                self.stop_robot()

                self.get_logger().error('Navigation timeout')

                goal_handle.abort()

                result = TravelNoCrashing.Result()
                return result

            traveled = self.get_traveled_distance()

            # Print only once per second
            if time.time() - last_log_time > 1.0:

                self.get_logger().info(
                    f'Traveled: {traveled:.2f}/{target_distance:.2f} m'
                )

                last_log_time = time.time()

            # Goal reached
            if traveled >= target_distance:

                self.stop_robot()

                self.get_logger().info('Goal reached')

                break

            # Read danger values
            front_left = self.danger_zones.front_left
            front_right = self.danger_zones.front_right

            front_danger = max(front_left, front_right)

            twist = Twist()

            # -------------------------------------------------
            # COLLISION AVOIDANCE
            # -------------------------------------------------

            # Critical danger
            if front_danger >= 3:

                # IMPORTANT:
                # move slightly forward while turning
                # otherwise robot may spin forever

                twist.linear.x = 0.05

                if front_left >= front_right:
                    twist.angular.z = -0.6
                else:
                    twist.angular.z = 0.6

            # Medium danger
            elif front_danger == 2:

                twist.linear.x = 0.12

                if front_left >= front_right:
                    twist.angular.z = -0.3
                else:
                    twist.angular.z = 0.3

            # Low danger
            elif front_danger == 1:

                twist.linear.x = 0.18
                twist.angular.z = 0.0

            # No danger
            else:

                twist.linear.x = 0.25
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)

            # -------------------------------------------------
            # FEEDBACK EVERY 3 SECONDS
            # -------------------------------------------------

            now = time.time()

            if now - last_feedback_time >= 3.0:

                elapsed = now - start_time

                secs = int(elapsed)
                nanosec = int((elapsed - secs) * 1e9)

                feedback_msg.traveled_distance = traveled

                feedback_msg.time_traveled = Duration(
                    sec=secs,
                    nanosec=nanosec
                )

                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(
                    f'Feedback sent: {traveled:.2f} m'
                )

                last_feedback_time = now

            # ROS-friendly sleep
            time.sleep(0.05)

        # -------------------------------------------------
        # RESULT
        # -------------------------------------------------

        result = TravelNoCrashing.Result()

        result.traveled_distance = self.get_traveled_distance()
        result.most_dangerous_zone = self.most_dangerous
        result.highest_danger_value = self.highest_danger

        goal_handle.succeed()

        self.get_logger().info('Action succeeded')

        return result


def main(args=None):

    rclpy.init(args=args)

    node = CollisionAvoiderNode()

    executor = MultiThreadedExecutor(num_threads=4)

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()