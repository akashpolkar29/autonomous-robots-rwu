import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_interfaces.msg import DangerZones
from custom_interfaces.srv import SetZoneSize
import math

class CollisionDetectorNode(Node):
    def __init__(self):
        super().__init__('collision_detector_node')

        # Default zone sizes
        self.critical = 0.5
        self.warn = 0.75
        self.safe = 1.0

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(DangerZones, '/danger_zones', 10)
        self.service = self.create_service(
            SetZoneSize, '/set_zone_size', self.set_zone_size_callback)

        self.get_logger().info('collision_detector_node started')

    def get_danger_level(self, distance):
        if distance <= self.critical:
            return 3  # red
        elif distance <= self.warn:
            return 2  # yellow
        elif distance <= self.safe:
            return 1  # green
        else:
            return 0  # white

    def get_min_distance(self, ranges, start_idx, end_idx):
        sector = ranges[start_idx:end_idx]
        valid = [r for r in sector if not math.isinf(r) and not math.isnan(r)]
        return min(valid) if valid else float('inf')

    def scan_callback(self, msg):
        ranges = msg.ranges
        total = len(ranges)

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

    def set_zone_size_callback(self, request, response):
        # Validate: critical < warn < safe
        if request.critical < request.warn < request.safe:
            self.critical = request.critical
            self.warn = request.warn
            self.safe = request.safe
            response.success = True
            response.message = (
                f'Zone sizes updated: critical={self.critical}, '
                f'warn={self.warn}, safe={self.safe}'
            )
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Invalid zone sizes: must satisfy critical < warn < safe'
            self.get_logger().warn(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()