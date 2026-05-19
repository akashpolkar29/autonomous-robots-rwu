import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_interfaces.msg import DangerZones
import math

class CollisionDetectorNode(Node):
    def __init__(self):
        super().__init__('collision_detector_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(DangerZones, '/danger_zones', 10)

    def get_danger_level(self, distance):
        if distance <= 0.5:
            return 3  # red
        elif distance <= 0.75:
            return 2  # yellow
        elif distance <= 1.0:
            return 1  # green
        else:
            return 0  # white (safe)

    def get_min_distance(self, ranges, start_idx, end_idx):
        sector = ranges[start_idx:end_idx]
        valid = [r for r in sector if not math.isinf(r) and not math.isnan(r)]
        return min(valid) if valid else float('inf')

    def scan_callback(self, msg):
        ranges = msg.ranges
        total = len(ranges)  # should be ~360

        # Each quadrant is 90° = ~90 indices
        front_left_dist  = self.get_min_distance(ranges, 0, 45)
        front_right_dist = self.get_min_distance(ranges, total - 45, total)
        back_left_dist   = self.get_min_distance(ranges, 135, 180)
        back_right_dist  = self.get_min_distance(ranges, 180, 225)

        danger = DangerZones()
        danger.front_left  = self.get_danger_level(front_left_dist)
        danger.front_right = self.get_danger_level(front_right_dist)
        danger.back_left   = self.get_danger_level(back_left_dist)
        danger.back_right  = self.get_danger_level(back_right_dist)

        self.publisher.publish(danger)
        self.get_logger().info(
            f'FL:{danger.front_left} FR:{danger.front_right} '
            f'BL:{danger.back_left} BR:{danger.back_right}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()