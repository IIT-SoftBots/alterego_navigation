#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from alterego_msgs.srv import NavService


class NavServiceNode(Node):
    def __init__(self):
        super().__init__('alterego_nav_service')
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        self.declare_parameter('navigation_locations_hier', None)
        self.locations = self.get_parameter('navigation_locations_hier').value
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.service = self.create_service(NavService, 'navigation_service', self.handle_request)

    def handle_request(self, request, response):
        self.get_logger().info(f'Received request: {request.location} - {request.sub_location}')
        # Lookup position/orientation in hierarchical list structure (list[ {Location: [ {Sub: {...}}, ... ]} ])
        pose_data = self.lookup_pose(request.location, request.sub_location)
        if pose_data is None:
            self.get_logger().error('Requested location not found.')
            response.success = False
            return response
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action unavailable.')
            response.success = False
            return response
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose_data['position']['x']
        goal.pose.pose.position.y = pose_data['position']['y']
        goal.pose.pose.position.z = pose_data['position']['z']
        goal.pose.pose.orientation.x = pose_data['orientation']['x']
        goal.pose.pose.orientation.y = pose_data['orientation']['y']
        goal.pose.pose.orientation.z = pose_data['orientation']['z']
        goal.pose.pose.orientation.w = pose_data['orientation']['w']
        send_future = self.action_client.send_goal_async(goal)
        def goal_resp(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected')
                response.success = False
                return
            result_future = goal_handle.get_result_async()
            def done_cb(rf):
                # TODO inspect rf.result().result for status code if needed
                response.success = True
            result_future.add_done_callback(done_cb)
        send_future.add_done_callback(goal_resp)
        # NOTE: We return immediately; for synchronous service semantics you'd need to spin until result.
        response.success = True  # optimistic until detailed result handled
        return response

    def lookup_pose(self, loc, sub):
        data = self.locations
        if data is None:
            return None
        try:
            for entry in data:  # each entry: {Location: [ {SubLocation: {...}}, ...] }
                if loc in entry:
                    for subentry in entry[loc]:
                        if sub in subentry:
                            return subentry[sub]
        except Exception as e:
            self.get_logger().error(f'Error parsing locations parameter: {e}')
        return None


def main():
    rclpy.init()
    node = NavServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()