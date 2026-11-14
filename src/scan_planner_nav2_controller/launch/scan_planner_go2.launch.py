"""Run SCAN-Planner against the existing FAST-LIO and Go2 simulation topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    planner = Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        output="screen",
        parameters=[
            planner_yaml,
            {
                "use_sim_time": use_sim_time,
                "fsm.navi_mode": 3,
                "fsm.odom_twist_in_body_frame": True,
                "grid_map.frame_id": "map",
                "grid_map.sensor_type": "lidar",
                "grid_map.cloud_is_world": True,
                "grid_map.need_extrinsic": False,
                "grid_map.double_cylinder_radius": 0.21,
                "grid_map.double_cylinder_offset": 0.10,
                "optimization.dist0": 0.01,
                "manager.max_vel": 0.30,
                "manager.max_acc": 0.20,
                "optimization.max_vel": 0.30,
                "optimization.max_acc": 0.20,
            },
        ],
        remappings=[
            ("body_pose", "/fastlio/odom"),
            ("sensor_pose", "/fastlio/odom"),
            ("cloud", "/cloud_registered"),
            ("initial_path", "/scanplanner/initial_path"),
        ],
    )

    controller = Node(
        package="scan_planner",
        executable="closed_loop_controller",
        name="closed_loop_controller",
        output="screen",
        parameters=[
            controllers_yaml,
            {
                "use_sim_time": use_sim_time,
                "max_vx": 0.30,
                "max_vy": 0.18,
                "max_vyaw": 0.35,
            },
        ],
        remappings=[
            ("body_pose", "/fastlio/odom"),
            ("cmd_vel", "/scanplanner/cmd_vel"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            planner,
            controller,
        ]
    )
