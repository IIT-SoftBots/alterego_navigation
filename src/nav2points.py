#!/usr/bin/env python3
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class Nav2Points(Node):
    def __init__(self):
        super().__init__('alterego_nav2points')
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        # Parameters: list of dicts with locations
        self.declare_parameter('navigation_locations', None)
        self.locations = self.get_parameter('navigation_locations').get_parameter_value().string_array_value if \
            self.get_parameter('navigation_locations').value is not None else None
        # In ROS1 this was a structured list; here we expect it to be set via YAML as a parameter (list of dictionaries not directly supported as-is). For transition we will later parse using a separate parameter loader.
        self.goal_reached_pub = self.create_publisher(String, 'goal_reached', 10)
        self.create_subscription(String, 'target_location', self.target_callback, 10)
        self.create_subscription(Path, 'local_plan', self.path_callback, 10)
        self.path_length = None
        self.goal_in_progress = False
        self.goal_reached_published = False
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def path_callback(self, msg: Path):
        self.path_length = self.calculate_path_length(msg)

    def calculate_path_length(self, path: Path):
        length = 0.0
        poses = path.poses
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            dx = p1.x - p2.x
            dy = p1.y - p2.y
            length += math.sqrt(dx * dx + dy * dy)
        return length

    def target_callback(self, msg: String):
        target = msg.data
        self.get_logger().info(f'Received target location: {target}')
        # Resolve target into position/orientation; placeholder expects parameters loaded into a dict param 'navigation_locations_dict'
        nav_dict_param = self.get_parameter_or('navigation_locations_dict', None).value
        if nav_dict_param is None:
            self.get_logger().error('navigation_locations_dict parameter not set (expected serialized YAML).')
            return
        # Expect a YAML serialized string we can eval safely? For safety we skip eval and expect not implemented.
        # TODO: implement real parsing if needed.
        self.get_logger().warn('Location resolution not yet implemented in ROS2 conversion.')
        # Early exit until mapping provided
        return

    def send_goal(self, position, orientation):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available.')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = position['x']
        goal_msg.pose.pose.position.y = position['y']
        goal_msg.pose.pose.position.z = position['z']
        goal_msg.pose.pose.orientation.x = orientation['x']
        goal_msg.pose.pose.orientation.y = orientation['y']
        goal_msg.pose.pose.orientation.z = orientation['z']
        goal_msg.pose.pose.orientation.w = orientation['w']
        self.get_logger().info('Sending NavigateToPose goal')
        send_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        # Could inspect feedback.feedback for distance remaining
        if self.path_length is not None and self.path_length < 1.5 and not self.goal_reached_published:
            msg = String()
            msg.data = 'SUCCEEDED'
            self.goal_reached_pub.publish(msg)
            self.goal_reached_published = True
            self.get_logger().info('Goal is within threshold (path length).')

    def result_callback(self, future):
        result = future.result().result
        # Nav2 result has result code in status (not in this minimal stub). We assume success for now.
        msg = String()
        msg.data = 'SUCCEEDED'
        self.goal_reached_pub.publish(msg)
        self.get_logger().info('Goal reached (result callback).')


def main():
    rclpy.init()
    node = Nav2Points()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()