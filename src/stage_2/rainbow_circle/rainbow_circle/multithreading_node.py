import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen


rainbow_colors = [
    (255, 0,   0),    # red
    (255, 127, 0),    # orange
    (255, 255, 0),    # yellow
    (0,   255, 0),    # green
    (0,   0,   255),  # blue
    (75,  0,   130),  # indigo
    (148, 0,   211),  # violet
]


class MultiThreadingNode(Node):

    def __init__(self):
        super().__init__('multithreading_node')

        self.color_index = 0

        self.timer_group  = MutuallyExclusiveCallbackGroup()
        self.client_group = MutuallyExclusiveCallbackGroup()

        self.set_pen_client = self.create_client(
            SetPen,
            '/turtle1/set_pen',
            callback_group=self.client_group
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.main_timer = self.create_timer(
            0.2,
            self.loop,
            callback_group=self.timer_group
        )

        self.get_logger().info('MultiThreadingNode started')

    def loop(self):
        # Drive in a circle
        twist = Twist()
        twist.linear.x  = 2.0
        twist.angular.z = 2.0
        self.cmd_vel_pub.publish(twist)

        # Cycle color
        r, g, b = rainbow_colors[self.color_index]
        self.color_index = (self.color_index + 1) % len(rainbow_colors)

        req = SetPen.Request()
        req.r     = r
        req.g     = g
        req.b     = b
        req.width = 4
        req.off   = 0

        self.get_logger().info(f'Calling set_pen with color ({r}, {g}, {b})')
        self.set_pen_client.call(req)


def main(args=None):
    rclpy.init(args=args)
    node = MultiThreadingNode()

    # Wait for set_pen service before starting
    node.set_pen_client.wait_for_service()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()