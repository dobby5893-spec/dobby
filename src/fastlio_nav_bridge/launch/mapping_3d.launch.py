"""Start FAST-LIO and an OctoMap that uses the moving LiDAR origin."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    mapping_launch = os.path.join(
        fast_lio_share, 'launch', 'mapping.launch.py')
    mid360_config = os.path.join(
        fast_lio_share, 'config', 'mid360.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('resume_state', default_value=''),
        DeclareLaunchArgument('octomap_path', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz': 'false',
                'config_file': 'mid360.yaml',
            }.items(),
        ),
        Node(
            package='fastlio_nav_bridge',
            executable='fastlio_tf_bridge',
            name='fastlio_tf_bridge',
            parameters=[{'use_sim_time': True, 'resume_state': ParameterValue(LaunchConfiguration('resume_state'), value_type=str)}],
            output='screen',
        ),
        Node(
            package='fastlio_nav_bridge',
            executable='fastlio_lidar_frame_relay',
            name='fastlio_lidar_frame_relay',
            parameters=[mid360_config, {
                'use_sim_time': True,
                'publish_every_n': 5,
                'wait_for_alignment': True,
            }],
            output='screen',
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server_3d',
            parameters=[{
                'use_sim_time': True,
                'frame_id': 'map',
                'octomap_path': ParameterValue(LaunchConfiguration('octomap_path'), value_type=str),
                'base_frame_id': 'body',
                # Preserve doorway gaps in the obstacle map as well as the
                # pose search. A 10 cm voxel can consume a 5 cm clearance.
                'resolution': 0.05,
                'sensor_model.max_range': 8.0,
                'point_cloud_min_z': -0.5,
                'point_cloud_max_z': 6.0,
                'occupancy_min_z': 0.20,
                'occupancy_max_z': 1.60,
                'filter_ground_plane': False,
                'filter_speckles': True,
                'incremental_2D_projection': False,
                'latch': True,
            }],
            remappings=[
                ('cloud_in', '/fastlio/lidar_octomap'),
                ('projected_map', '/map'),
            ],
            output='screen',
        ),
    ])
