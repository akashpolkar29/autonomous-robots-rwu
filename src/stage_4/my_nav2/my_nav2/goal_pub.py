import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_pub')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.publish_goal()

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 1.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Goal published!')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    node.destroy_node()
    rclpy.shutdown()