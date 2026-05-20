import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_interfaces.msg import DangerZones
from custom_interfaces.srv import SetZoneSize


class CollisionDetectorNode(Node):

    def __init__(self):
        super().__init__('collision_detector_node')

        self.critical = 0.5
        self.warn     = 0.75
        self.safe     = 1.0

        self.publisher = self.create_publisher(DangerZones, '/danger_zones', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.service = self.create_service(
            SetZoneSize, '/set_zone_size', self.set_zone_size_callback)

        self.get_logger().info('CollisionDetectorNode started')

    def set_zone_size_callback(self, request, response):
        self.critical = request.critical
        self.warn     = request.warn
        self.safe     = request.safe
        self.get_logger().info(
            f'Zones updated: critical={self.critical} warn={self.warn} safe={self.safe}')
        response.success = True
        return response

    def get_danger_level(self, distance):
        if distance <= self.critical:
            return 3
        elif distance <= self.warn:
            return 2
        elif distance <= self.safe:
            return 1
        else:
            return 0

    def scan_callback(self, msg):
        ranges  = msg.ranges
        total   = len(ranges)
        quarter = total // 4

        def min_valid(r):
            valid = [x for x in r if x > 0.01 and x == x]
            return min(valid) if valid else 999.0

        msg_out = DangerZones()
        msg_out.front_left  = self.get_danger_level(min_valid(ranges[:quarter]))
        msg_out.front_right = self.get_danger_level(min_valid(ranges[3*quarter:]))
        msg_out.back_left   = self.get_danger_level(min_valid(ranges[quarter:2*quarter]))
        msg_out.back_right  = self.get_danger_level(min_valid(ranges[2*quarter:3*quarter]))

        self.get_logger().info(
            f'Zones -> FL:{msg_out.front_left} FR:{msg_out.front_right} '
            f'BL:{msg_out.back_left} BR:{msg_out.back_right}')

        self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()