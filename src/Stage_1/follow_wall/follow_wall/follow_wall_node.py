import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowWall(Node):

    def __init__(self):
        super().__init__('follow_wall_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        ranges = msg.ranges

        front = ranges[0]
        right = ranges[270]   # right side

        self.get_logger().info(f"Front: {front}, Right: {right}")

        twist = Twist()

        # Avoid crash
        if front < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # turn left

        else:
            twist.linear.x = 0.2

            # Maintain wall distance
            if right > 0.5:
                twist.angular.z = -0.3  # go right
            elif right < 0.5:
                twist.angular.z = 0.3   # go left
            else:
                twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()