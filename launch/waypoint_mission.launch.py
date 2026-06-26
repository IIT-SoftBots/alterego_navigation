#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('alterego_navigation')
    default_waypoints = os.path.join(pkg_share, 'config', 'example_waypoints.yaml')

    namespace_arg = DeclareLaunchArgument('namespace', default_value='alterego5')
    waypoints_file_arg = DeclareLaunchArgument('waypoints_file', default_value=default_waypoints)

    waypoint_runner = Node(
        package='alterego_navigation',
        executable='waypoint_mission',
        name='waypoint_mission',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'global_frame': 'map',
                'action_name': 'follow_waypoints',
            }
        ],
    )

    return LaunchDescription([
        namespace_arg,
        waypoints_file_arg,
        waypoint_runner,
    ])
