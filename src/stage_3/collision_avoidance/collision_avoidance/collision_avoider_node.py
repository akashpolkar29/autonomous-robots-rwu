import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import time

from custom_interfaces.action import TravelNoCrashing
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

# ---- Import your existing DangerZones message ----
# Adjust this import to match your actual message type!
from custom_interfaces.msg import DangerZones  # <-- check your msg folder name


class CollisionAvoiderNode(Node):

    def __init__(self):
        super().__init__('collision_avoider_node')

        self._cb_group = ReentrantCallbackGroup()

        # Action server
        self._action_server = ActionServer(
            self,
            TravelNoCrashing,
            '/drive_no_crashing',
            self.execute_callback,
            callback_group=self._cb_group
        )

        # Odometry subscriber
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self._cb_group
        )

        # Danger zones subscriber
        self._danger_sub = self.create_subscription(
            DangerZones,
            '/danger_zones',
            self.danger_callback,
            10,
            callback_group=self._cb_group
        )

        # Velocity publisher
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self._current_x = None
        self._current_y = None
        self._danger_zones = None
        self._most_dangerous_zone = ''
        self._highest_danger_value = 0

        self.get_logger().info('CollisionAvoiderNode ready.')

    def odom_callback(self, msg: Odometry):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y

    def danger_callback(self, msg):
        # Adjust this logic to your actual DangerZones message fields!
        # This assumes your message has a list of zones with a name and danger value.
        self._danger_zones = msg
        highest = 0
        most_dangerous = ''
        # Example: msg.zones is a list with .name and .danger_value fields
        # Adjust to your actual message structure:
        for zone in msg.zones:
            if zone.danger_value > highest:
                highest = zone.danger_value
                most_dangerous = zone.name
        self._most_dangerous_zone = most_dangerous
        self._highest_danger_value = highest

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: travel {goal_handle.request.target_distance} m')

        # Wait until we have an odom fix
        while self._current_x is None:
            self.get_logger().info('Waiting for /odom...')
            time.sleep(0.1)

        start_x = self._current_x
        start_y = self._current_y
        start_time = self.get_clock().now()

        target = goal_handle.request.target_distance
        feedback_msg = TravelNoCrashing.Feedback()

        FORWARD_SPEED = 0.2       # m/s
        TURN_SPEED = 0.5          # rad/s
        DANGER_THRESHOLD = 50     # adjust to your danger scale
        FEEDBACK_INTERVAL = 3.0   # seconds

        last_feedback_time = time.time()
        rate_hz = 10
        loop_rate = 1.0 / rate_hz

        cmd = Twist()

        while True:
            # Calculate traveled distance
            dx = self._current_x - start_x
            dy = self._current_y - start_y
            traveled = math.sqrt(dx * dx + dy * dy)

            # Check if goal reached
            if traveled >= target:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self._cmd_pub.publish(cmd)
                break

            # Collision avoidance logic
            # Adapt this to your actual DangerZones message fields!
            danger_front = False
            if self._danger_zones is not None:
                for zone in self._danger_zones.zones:
                    if 'front' in zone.name.lower() and zone.danger_value > DANGER_THRESHOLD:
                        danger_front = True
                        break

            if danger_front:
                # Turn to avoid
                cmd.linear.x = 0.0
                cmd.angular.z = TURN_SPEED
            else:
                # Drive forward
                cmd.linear.x = FORWARD_SPEED
                cmd.angular.z = 0.0

            self._cmd_pub.publish(cmd)

            # Publish feedback every 3 seconds
            now = time.time()
            if now - last_feedback_time >= FEEDBACK_INTERVAL:
                elapsed = self.get_clock().now() - start_time
                elapsed_sec = int(elapsed.nanoseconds / 1e9)
                elapsed_nanosec = int(elapsed.nanoseconds % 1e9)

                dur = Duration()
                dur.sec = elapsed_sec
                dur.nanosec = elapsed_nanosec

                feedback_msg.traveled_distance = traveled
                feedback_msg.time_traveled = dur
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'Feedback: traveled {traveled:.2f} m')
                last_feedback_time = now

            time.sleep(loop_rate)

        # Set result
        goal_handle.succeed()
        result = TravelNoCrashing.Result()
        result.traveled_distance = traveled
        result.most_dangerous_zone = self._most_dangerous_zone
        result.highest_danger_value = self._highest_danger_value
        self.get_logger().info(f'Goal succeeded! Traveled {traveled:.2f} m')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoiderNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()