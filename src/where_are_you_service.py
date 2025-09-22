#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from alterego_msgs.srv import WhereRUService
from math import sqrt


class WhereAreYouService(Node):
    def __init__(self):
        super().__init__('alterego_where_are_you')
        self.declare_parameter('navigation_keypoints', None)
        self.key_points = self.get_parameter('navigation_keypoints').value or []
        self.current_pose = None
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_cb, 10)
        self.service = self.create_service(WhereRUService, 'where_are_you_service', self.handle)

    def pose_cb(self, msg):
        self.current_pose = msg

    def handle(self, request, response):
        if self.current_pose is None:
            self.get_logger().warn('No pose received yet.')
            response.success = False
            response.instruction_point = ''
            return response
        nearest = self.find_nearest_point()
        if nearest:
            response.success = True
            response.instruction_point = nearest['name']
            self.get_logger().info(f'Nearest point: {nearest["name"]}')
        else:
            response.success = False
            response.instruction_point = ''
        return response

    def find_nearest_point(self):
        min_d = float('inf')
        best = None
        try:
            for loc in self.key_points:
                for name, data in loc.items():
                    d = self.distance(self.current_pose.pose.pose.position, data['position'])
                    if d < min_d:
                        min_d = d
                        best = {'name': name}
        except Exception as e:
            self.get_logger().error(f'Error parsing key points: {e}')
        return best

    def distance(self, p, pos_dict):
        dx = p.x - pos_dict['x']
        dy = p.y - pos_dict['y']
        dz = p.z - pos_dict['z']
        return sqrt(dx*dx + dy*dy + dz*dz)


def main():
    rclpy.init()
    node = WhereAreYouService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()