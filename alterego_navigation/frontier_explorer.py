#!/usr/bin/env python3

import math
from collections import deque
from typing import List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


GridCell = Tuple[int, int]


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('planner_period_sec', 2.5)
        self.declare_parameter('min_frontier_size', 12)
        self.declare_parameter('stuck_timeout_sec', 90.0)

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        period = self.get_parameter('planner_period_sec').get_parameter_value().double_value

        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value
        self.stuck_timeout_sec = self.get_parameter('stuck_timeout_sec').get_parameter_value().double_value

        self.map_msg: Optional[OccupancyGrid] = None
        self.current_goal_start_time = None
        self.goal_active = False

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.on_map, 10)
        self.nav_client = ActionClient(self, NavigateToPose, action_name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info('Frontier explorer avviato.')

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def on_timer(self) -> None:
        if self.map_msg is None:
            return

        if self.goal_active and self.current_goal_start_time is not None:
            elapsed = (self.get_clock().now() - self.current_goal_start_time).nanoseconds / 1e9
            if elapsed > self.stuck_timeout_sec:
                self.get_logger().warn('Goal in timeout, richiedo cancellazione.')
                self.goal_active = False

        if self.goal_active:
            return
    #standing_pitch_offset: 0.065 # compensation moved at imu level (filter node), see imu.yaml

        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            return

        target = self.pick_frontier_target(robot_pose[0], robot_pose[1])
        if target is None:
            self.get_logger().info('Nessuna frontiera valida trovata.')
            return

        self.send_goal(target[0], target[1], robot_pose[0], robot_pose[1])

    def lookup_robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            self.get_logger().warn('Trasformazione map->base_link non disponibile.', throttle_duration_sec=5.0)
            return None

    def pick_frontier_target(self, robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        assert self.map_msg is not None

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        data = self.map_msg.data

        def idx(x: int, y: int) -> int:
            return y * width + x

        def is_in_bounds(x: int, y: int) -> bool:
            return 0 <= x < width and 0 <= y < height

        def is_frontier(x: int, y: int) -> bool:
            if data[idx(x, y)] != 0:
                return False
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if is_in_bounds(nx, ny) and data[idx(nx, ny)] == -1:
                    return True
            return False

        frontier_cells: Set[GridCell] = set()
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if is_frontier(x, y):
                    frontier_cells.add((x, y))

        if not frontier_cells:
            return None

        visited: Set[GridCell] = set()
        best_score = -1.0
        best_target = None

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = self.cluster_frontier(cell, frontier_cells, visited)
            if len(cluster) < self.min_frontier_size:
                continue

            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)

            world_x = origin_x + (cx + 0.5) * resolution
            world_y = origin_y + (cy + 0.5) * resolution

            dist = math.hypot(world_x - robot_x, world_y - robot_y)
            score = len(cluster) / (dist + 1.0)

            if score > best_score:
                best_score = score
                best_target = (world_x, world_y)

        return best_target

    def cluster_frontier(
        self,
        start: GridCell,
        frontier_cells: Set[GridCell],
        visited: Set[GridCell],
    ) -> List[GridCell]:
        queue = deque([start])
        cluster: List[GridCell] = []
        visited.add(start)

        while queue:
            x, y = queue.popleft()
            cluster.append((x, y))
            for nx in range(x - 1, x + 2):
                for ny in range(y - 1, y + 2):
                    ncell = (nx, ny)
                    if ncell in frontier_cells and ncell not in visited:
                        visited.add(ncell)
                        queue.append(ncell)

        return cluster

    def send_goal(self, target_x: float, target_y: float, robot_x: float, robot_y: float) -> None:
        if not self.nav_client.server_is_ready():
            self.get_logger().info('Attendo action server NavigateToPose...')
            self.nav_client.wait_for_server(timeout_sec=2.0)
            if not self.nav_client.server_is_ready():
                return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target_x)
        pose.pose.position.y = float(target_y)

        yaw = math.atan2(target_y - robot_y, target_x - robot_x)
        pose.pose.orientation = quaternion_from_yaw(yaw)

        goal.pose = pose

        self.goal_active = True
        self.current_goal_start_time = self.get_clock().now()

        self.get_logger().info(f'Invio goal frontiera: x={target_x:.2f}, y={target_y:.2f}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal frontiera rifiutato.')
            self.goal_active = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future) -> None:
        status = future.result().status
        self.get_logger().info(f'Goal frontiera terminato con status={status}')
        self.goal_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
