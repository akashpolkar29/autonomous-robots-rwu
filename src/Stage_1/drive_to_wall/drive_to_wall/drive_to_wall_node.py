import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class DriveToWall(Node):

    def __init__(self):
        super().__init__('drive_to_wall_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        distance = msg.ranges[0]

        twist = Twist()

        if distance > 1.0:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
