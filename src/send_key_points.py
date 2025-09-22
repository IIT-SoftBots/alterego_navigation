#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, atan2, degrees, acos, cos, sin


class KeyPointSender(Node):
    def __init__(self):
        super().__init__('alterego_keypoints')
        self.declare_parameter('navigation_keypoints', None)
        raw_kp = self.get_parameter('navigation_keypoints').value
        self.key_points = self.load_key_points(raw_kp) if raw_kp is not None else []
        self.nearest_pub = self.create_publisher(String, 'nearest_point', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'keypoint_markers_array', 10)
        self.current_pose = None
        self.threshold = 2.0
        self.angle_threshold = 60.0
        self.last_published_point = None
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def load_key_points(self, param):
        kp = []
        try:
            for location in param:
                for name, data in location.items():
                    kp.append({'name': name, 'position': data['position'], 'orientation': data['orientation']})
        except Exception as e:
            self.get_logger().error(f'Error parsing navigation_keypoints: {e}')
        return kp

    def pose_callback(self, msg):
        self.current_pose = msg

    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2['x']
        dy = pose1.position.y - pose2['y']
        dz = pose1.position.z - pose2['z']
        return sqrt(dx*dx + dy*dy + dz*dz)

    def quaternion_to_yaw(self, orientation):
        return atan2(2.0 * (orientation['w'] * orientation['z'] + orientation['x'] * orientation['y']),
                     1.0 - 2.0 * (orientation['y'] * orientation['y'] + orientation['z'] * orientation['z']))

    def calculate_angle(self, pose1, pose2, orientation2):
        dx = pose1.position.x - pose2['x']
        dy = pose1.position.y - pose2['y']
        keypoint_yaw = self.quaternion_to_yaw(orientation2)
        keypoint_dir_x = cos(keypoint_yaw)
        keypoint_dir_y = sin(keypoint_yaw)
        dot_product = dx * keypoint_dir_x + dy * keypoint_dir_y
        mag_robot = sqrt(dx**2 + dy**2)
        mag_keypoint = sqrt(keypoint_dir_x**2 + keypoint_dir_y**2)
        if mag_robot == 0 or mag_keypoint == 0:
            return 0.0
        return degrees(acos(dot_product / (mag_robot * mag_keypoint)))

    def find_point_within_threshold(self):
        if self.current_pose is None:
            return None
        for kp in self.key_points:
            distance = self.calculate_distance(self.current_pose.pose.pose, kp['position'])
            angle = self.calculate_angle(self.current_pose.pose.pose, kp['position'], kp['orientation'])
            if distance <= self.threshold and angle <= self.angle_threshold:
                return kp
        return None

    def normalize_quaternion(self, q):
        norm = sqrt(q['x']**2 + q['y']**2 + q['z']**2 + q['w']**2)
        return {k: q[k] / norm for k in q}

    def create_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, kp in enumerate(self.key_points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'keypoints'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = kp['position']['x']
            marker.pose.position.y = kp['position']['y']
            marker.pose.position.z = kp['position']['z']
            norm_q = self.normalize_quaternion(kp['orientation'])
            marker.pose.orientation.x = norm_q['x']
            marker.pose.orientation.y = norm_q['y']
            marker.pose.orientation.z = norm_q['z']
            marker.pose.orientation.w = norm_q['w']
            marker.scale.x = 1.0
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        return marker_array

    def timer_cb(self):
        point = self.find_point_within_threshold()
        if point and point != self.last_published_point:
            msg = String()
            msg.data = point['name']
            self.nearest_pub.publish(msg)
            self.last_published_point = point
            self.get_logger().info(f'Nearest point within threshold: {point["name"]}')
        self.marker_pub.publish(self.create_markers())


def main():
    rclpy.init()
    node = KeyPointSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()