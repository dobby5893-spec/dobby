import os
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_share = get_package_share_directory('go2_multifloor_exploration')
    params = os.path.join(package_share, 'config', 'multifloor.yaml')

    resume_path = LaunchConfiguration('resume_state').perform(context)
    manager_resume = {}
    slice_resume = {}
    if resume_path:
        state = json.loads(Path(resume_path).read_text())['floor']
        manager_resume = dict(resume_enabled=True, resume_floor_z=float(state['active_floor_z']),
                              resume_visited_floors=[float(x) for x in state['visited_floors']],
                              resume_phase=int(state['phase']),
                              resume_has_residuals=bool(state.get('has_residuals', False)))
        slice_resume = dict(initial_floor_z=float(state['active_floor_z']))
    return [
        Node(
            package='go2_multifloor_exploration',
            executable='floor_slice_node',
            name='floor_slice_node',
            parameters=[params, {'use_sim_time': use_sim_time}, slice_resume],
            output='screen',
        ),
        Node(
            package='go2_multifloor_exploration',
            executable='multifloor_manager',
            name='multifloor_manager',
            parameters=[params, {'use_sim_time': use_sim_time}, manager_resume],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('resume_state', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
