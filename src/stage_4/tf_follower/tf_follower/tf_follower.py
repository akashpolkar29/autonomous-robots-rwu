import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor


class TfFollower(Node):

    def __init__(self):
        super().__init__('tf_follower')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting TF follower.')

        self._current_goal_handle = None
        self._last_x = None
        self._last_y = None
        self._min_move_distance = 0.3

        # Poll the TF at 2Hz
        self.timer = self.create_timer(0.5, self.follow_target)

    def follow_target(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'target_pose',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get target_pose TF: {e}')
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        if self._last_x is not None:
            dist = ((tx - self._last_x) ** 2 + (ty - self._last_y) ** 2) ** 0.5
            if dist < self._min_move_distance:
                return

        self._last_x = tx
        self._last_y = ty

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tx
        pose.pose.position.y = ty
        pose.pose.orientation.w = 1.0

        goal_msg.pose = pose

        self.get_logger().info(f'Sending goal: x={tx:.2f}, y={ty:.2f}')

        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        self._current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted')


def main(args=None):
    rclpy.init(args=args)
    node = TfFollower()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()