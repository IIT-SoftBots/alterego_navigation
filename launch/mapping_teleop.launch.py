#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    pkg_share = get_package_share_directory('alterego_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    bringup_share = get_package_share_directory('alterego_bringup')

    namespace_arg = DeclareLaunchArgument('namespace', default_value='alterego5')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    alterego_version_arg = DeclareLaunchArgument('alterego_version', default_value='2')

    # FIXME: this is not working @Luca
    map_save_path_arg = DeclareLaunchArgument(
        'map_save_path',
        default_value='/tmp/alterego_map',
        description='Base path used by map_saver_cli when the launch shuts down',
    )

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')


    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            #'namespace': LaunchConfiguration('namespace'),
            #'use_namespace': LaunchConfiguration('use_namespace'),
            'slam': 'False',
            'use_localization': 'False',
            'map': '',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    # Launch slam_toolbox explicitly with AlterEGO mapping params.
    # This avoids Nav2 bringup defaults overriding base_frame/scan_topic.
    slam_toolbox = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_share, 'launch', 'online_sync_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam_params_file': slam_params_file,
                }.items(),
            ),
        ]
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
        #namespace_arg,
        #use_namespace_arg,
        use_sim_time_arg,
        autostart_arg,
        #alterego_version_arg,
        map_save_path_arg,
        bringup,
        slam_toolbox,
        save_map_on_shutdown,
    ])
