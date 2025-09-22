#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Timer single-shot like substitute: publish once after short delay
        self.timer_ = self.create_timer(0.5, self.publish_once)
        self.done_ = False

    def publish_once(self):
        if self.done_:
            return
        # Parameter retrieval: expected to be provided via parameter or YAML
        # In ROS1 it used get_param('navigation/Locations'); here we expose a parameter 'navigation_locations'
        locations = self.get_parameter_or('navigation_locations', None).value
        if locations is None:
            self.get_logger().error('Parameter navigation_locations not set (expected list of dicts).')
            self.done_ = True
            return
        ingresso_data = None
        for location in locations:
            if 'Ingresso' in location:
                ingresso_data = location['Ingresso']
                break
        if ingresso_data is None:
            self.get_logger().error('Ingresso not found in navigation_locations')
            self.done_ = True
            return
        position = ingresso_data['position']
        orientation = ingresso_data['orientation']
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = Pose(
            position=Point(x=position['x'], y=position['y'], z=position['z']),
            orientation=Quaternion(x=orientation['x'], y=orientation['y'], z=orientation['z'], w=orientation['w'])
        )
        msg.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        ]
        self.publisher_.publish(msg)
        self.get_logger().info('Published initial pose for Ingresso')
        self.done_ = True
        # Optionally shutdown after publish
        self.destroy_timer(self.timer_)


def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=2.0)  # allow timer to fire
    # Keep spinning a little in case of late parameter set
    while rclpy.ok() and not node.done_:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()