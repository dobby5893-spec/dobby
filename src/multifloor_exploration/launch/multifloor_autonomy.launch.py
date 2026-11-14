"""Bring up mapping, Nav2, frontier exploration, and floor coordination."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    scan_root = '/home/dobby/scan'
    mapping_launch = os.path.join(
        get_package_share_directory('fastlio_nav_bridge'),
        'launch', 'mapping_3d.launch.py')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py')
    frontier_params = os.path.join(
        scan_root, 'src', 'frontier_exploration_ros2',
        'config', 'params.yaml')
    nav2_params = os.path.join(
        scan_root, 'config', 'nav2_go2_fastlio.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch)),
        Node(
            package='fastlio_nav_bridge',
            executable='fastlio_tf_bridge',
            name='fastlio_tf_bridge',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': nav2_params,
            }.items(),
        ),
        Node(
            package='frontier_exploration_ros2',
            executable='frontier_explorer',
            name='frontier_explorer',
            parameters=[
                frontier_params,
                {
                    'use_sim_time': True,
                    'autostart': False,
                    'control_service_enabled': True,
                    'completion_event_enabled': True,
                    'return_to_start_on_complete': False,
                    'exploration_boundary_polygon': [
                        -9.65, -6.65,
                        9.65, -6.65,
                        9.65, 6.65,
                        -9.65, 6.65,
                    ],
                },
            ],
            output='screen',
        ),
        Node(
            package='multifloor_exploration',
            executable='multifloor_coordinator',
            name='multifloor_coordinator',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
