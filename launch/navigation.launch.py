#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('alterego_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    namespace_arg = DeclareLaunchArgument('namespace', default_value='alterego5')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'erzelli.yaml'),
        description='Path to the map file (YAML) to load',
    )


    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # 'namespace': LaunchConfiguration('namespace'),
            # 'use_namespace': LaunchConfiguration('use_namespace'),
            #'slam': 'False',
            'use_localization': 'True',
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params_file,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    return LaunchDescription([
        # namespace_arg,
        # use_namespace_arg,
        use_sim_time_arg,
        autostart_arg,
        map_arg,
        bringup,
    ])
