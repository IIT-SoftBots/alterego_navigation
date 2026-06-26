#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('alterego_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    bringup_share = get_package_share_directory('alterego_bringup')

    namespace_arg = DeclareLaunchArgument('namespace', default_value='alterego5')
    alterego_version_arg = DeclareLaunchArgument('alterego_version', default_value='2')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    map_save_path_arg = DeclareLaunchArgument(
        'map_save_path',
        default_value='/tmp/alterego_map',
        description='Base path used by map_saver_cli when the launch shuts down',
    )

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    # Launch lidar
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
    )

    # Launch IMU
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'imu.launch.py')
        ),
    )

    # Launch wheels
    wheels = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'wheels.launch.py')
        ),
    )

    # Launch Nav2 bringup with SLAM enabled
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_namespace': LaunchConfiguration('use_namespace'),
            'slam': 'True',
            'map': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'slam_params_file': slam_params_file,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    # The explorer node picks frontier goals from the /map topic, so it must be launched after the bringup that includes the SLAM node
    explorer = Node(
        package='alterego_navigation',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'map_topic': 'map',
                'global_frame': 'map',
                'base_frame': 'base_link',
                'action_name': 'navigate_to_pose',
                'planner_period_sec': 2.5,
                'min_frontier_size': 12,
            }
        ],
    )

    save_map_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                        '-f', LaunchConfiguration('map_save_path'),
                    ],
                    output='screen',
                )
            ]
        )
    )

    return LaunchDescription([
        namespace_arg,
        use_namespace_arg,
        use_sim_time_arg,
        autostart_arg,
        alterego_version_arg,
        map_save_path_arg,
        SetEnvironmentVariable('ROBOT_NAME', LaunchConfiguration('namespace')),
        lidar,
        imu,
        wheels,
        bringup,
        explorer,
        save_map_on_shutdown,
    ])
