#!/usr/bin/env python3

import math
from typing import List

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WaypointMission(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_mission')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('action_name', 'follow_waypoints')

        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value

        if not self.waypoints_file:
            self.get_logger().error('Parametro waypoints_file non valorizzato.')
            raise RuntimeError('waypoints_file mancante')

        self.client = ActionClient(self, FollowWaypoints, action_name)

        self.get_logger().info(f'Caricamento waypoint da: {self.waypoints_file}')
        poses = self.load_waypoints(self.waypoints_file)

        self.get_logger().info('Attendo action server FollowWaypoints...')
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('FollowWaypoints non disponibile.')
            raise RuntimeError('FollowWaypoints non disponibile')

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info(f'Invio missione con {len(poses)} waypoint.')
        send_future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)

    def load_waypoints(self, file_path: str) -> List[PoseStamped]:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        entries = data.get('waypoints', [])
        if not entries:
            raise RuntimeError('Nessun waypoint definito nel file YAML.')

        now = self.get_clock().now().to_msg()
        poses: List[PoseStamped] = []
        for i, item in enumerate(entries):
            x = float(item['x'])
            y = float(item['y'])
            yaw = float(item.get('yaw', 0.0))

            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = quaternion_from_yaw(yaw)
            poses.append(pose)

            self.get_logger().info(f'Waypoint {i}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        return poses

    def on_feedback(self, feedback_msg) -> None:
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Waypoint corrente: {current}')

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Missione waypoint rifiutata.')
            rclpy.shutdown()
            return

        self.get_logger().info('Missione waypoint accettata.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future) -> None:
        result = future.result().result
        missed = list(result.missed_waypoints)
        if missed:
            self.get_logger().warn(f'Waypoint non raggiunti: {missed}')
        else:
            self.get_logger().info('Missione completata: tutti i waypoint raggiunti.')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMission()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
