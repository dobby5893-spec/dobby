"""Launch the complete two-floor Go2 exploration simulation with one command."""

import os
import json
import math
import tempfile
import yaml
from pathlib import Path
from fastlio_nav_bridge.checkpoint_store import load_checkpoint, world_fingerprint
from fastlio_nav_bridge.continuous_ground import (continuous_ground_region,
    needs_fresh_ground_alignment, read_alignment_region)

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile


def validate_workspace_packages():
    """Refuse mixed installations before starting any simulator processes."""
    workspace = Path('/home/dobby/scan')
    packages = (
        'go2_multifloor_exploration', 'frontier_exploration_ros2',
        'go2_surface_3d_planner', 'scan_planner_nav2_controller',
        'go2_nav2_recovery', 'fastlio_nav_bridge', 'fast_lio',
    )
    resolved = {}
    for package in packages:
        actual = Path(get_package_prefix(package)).resolve()
        expected = (workspace / 'install' / package).resolve()
        if actual != expected:
            raise RuntimeError(
                f'Mixed workspace: {package} resolves to {actual}; expected {expected}. '
                'Use /home/dobby/scan/tools/start_fresh_multifloor.sh to start with a clean environment.')
        resolved[package] = str(actual)
    return resolved


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def resolved_ground_actions(event, world, alignment_path, dependent_factory):
    if event.returncode != 0:
        raise RuntimeError('Fresh ground alignment failed; dependent navigation was not started')
    prior = read_alignment_region(alignment_path, world)
    print('Using synchronized fresh-start first-floor slab prior')
    return dependent_factory(prior)


def ground_parameter_file(source, prior):
    """Embed the array in each concrete node config, across Include/Timer scopes."""
    if not prior:
        return source
    with open(source) as stream:
        config = yaml.safe_load(stream)
    def inject(value):
        if not isinstance(value, dict):
            return
        if isinstance(value.get('ros__parameters'), dict):
            value['ros__parameters']['continuous_floor_region'] = list(prior)
        for child in list(value.values()):
            inject(child)
    inject(config)
    with tempfile.NamedTemporaryFile(mode='w', prefix='go2_ground_nodes_', suffix='.yaml', delete=False) as stream:
        json.dump(config, stream)
        return stream.name


def physics_process_exited(event, context):
    if context.is_shutdown or not event.cmd or Path(event.cmd[0]).name != 'gzserver':
        return []
    reason = f'Gazebo physics exited with code {event.returncode}; stopping exploration and navigation'
    return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]


def launch_setup(context):
    validate_workspace_packages()
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    navigation_rviz = LaunchConfiguration("navigation_rviz")
    mapping_rviz = LaunchConfiguration("mapping_rviz")
    record_trial = LaunchConfiguration("record_trial")
    start_exploration = LaunchConfiguration("start_exploration")

    checkpoint_dir = LaunchConfiguration('checkpoint_dir').perform(context)
    mode = LaunchConfiguration('resume').perform(context).lower()
    world_path = world.perform(context)
    root = Path(checkpoint_dir).expanduser()
    exists = (root / 'LATEST').exists() or any(root.glob('generation-*'))
    state = None
    resume_state = ''
    octomap_path = ''
    if mode not in ('auto', 'true', 'false'):
        raise RuntimeError('resume must be auto, true or false')
    if mode == 'true' or (mode == 'auto' and exists):
        directory, state = load_checkpoint(root, world_fingerprint(world_path))
        resume_state = str(directory / 'state.json')
        octomap_path = str(directory / 'map.ot')
        print('RESUMING exploration checkpoint: ' + str(directory))
    elif mode == 'false' and exists:
        raise RuntimeError('Use a different checkpoint_dir for a new task; existing progress will not be overwritten')
    spawn = {}
    if state:
        position, q = state['world_body']
        yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
        spawn = dict(world_init_x=str(position[0]), world_init_y=str(position[1]),
                     world_init_z=str(position[2]+0.06), world_init_heading=str(yaw))

    navigation_params = "/home/dobby/scan/config/nav2_go2_fastlio.yaml"
    navigation_rviz_config = "/home/dobby/scan/config/nav2_multifloor.rviz"
    mapping_rviz_config = (
        "/home/dobby/scan/install/fast_lio/share/fast_lio/rviz/fastlio.rviz"
    )
    frontier_params = (
        "/home/dobby/scan/src/frontier_exploration_ros2/config/params.yaml"
    )
    recorder = "/home/dobby/scan/tools/record_navigation.py"

    gazebo = include(
        "go2_config",
        "gazebo.launch.py",
        {
            "world": world,
            "use_sim_time": use_sim_time,
            "rviz": "false",
            "gui": gazebo_gui,
            **spawn,
        },
    )

    mapping = include("fastlio_nav_bridge", "mapping_3d.launch.py", {"resume_state": resume_state, "octomap_path": octomap_path})
    floors = include(
        "go2_multifloor_exploration",
        "multifloor_exploration.launch.py",
        {"use_sim_time": use_sim_time, "resume_state": resume_state, "require_startup_ready": "true",
         "exploration_enabled": start_exploration},
    )

    def navigation_action(params):
        return GroupAction(actions=[
            SetParameter(name="bond_timeout", value=30.0),
            include("nav2_bringup", "navigation_launch.py", {
                "use_sim_time": use_sim_time, "autostart": "true", "params_file": params}),
        ])

    scan_planner = include(
        "scan_planner_nav2_controller", "scan_planner_go2.launch.py",
        {"use_sim_time": use_sim_time})

    def frontier_action(params):
        return include("frontier_exploration_ros2", "frontier_explorer.launch.py", {
            "use_sim_time": use_sim_time, "autostart": "false",
            "control_service_enabled": "true", "managed_floor_sessions": "true", "params_file": params,
            "log_level": "info"})

    nav_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_navigation",
        output="screen",
        arguments=["-d", navigation_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(navigation_rviz),
    )
    pointcloud_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_fastlio_3d",
        output="screen",
        arguments=["-d", mapping_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(mapping_rviz),
    )
    trial_recorder = ExecuteProcess(
        cmd=["python3", recorder, "--ros-args", "-p", ["use_sim_time:=", use_sim_time]],
        output="screen",
        condition=IfCondition(record_trial),
    )

    checkpoint = Node(
        package='fastlio_nav_bridge', executable='exploration_checkpoint',
        name='exploration_checkpoint', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'checkpoint_dir': checkpoint_dir,
                     'world': world_path, 'resume_state': resume_state, 'managed_floor_sessions': True,
                     'start_exploration': start_exploration.perform(context).lower() == 'true'}])
    ground_enabled = LaunchConfiguration('continuous_ground_floor').perform(context).lower() == 'true'

    def dependent_actions(prior):
        parameters = []
        if prior:
            # A file preserves a double-array parameter on Humble.
            with tempfile.NamedTemporaryFile(mode='w', prefix='go2_floor_prior_', suffix='.yaml', delete=False) as stream:
                json.dump({'/**': {'ros__parameters': {'continuous_floor_region': prior}}}, stream)
                parameters = [SetParametersFromFile(filename=stream.name)]
        # Included navigation/exploration receive explicit merged node files.
        # Direct floor nodes inherit the same prior through this scope.
        return [GroupAction(actions=parameters + [
            floors,
            TimerAction(period=4., actions=[navigation_action(ground_parameter_file(navigation_params, prior))]),
            TimerAction(period=7., actions=[scan_planner]),
            TimerAction(period=10., actions=[nav_rviz, pointcloud_rviz, trial_recorder, checkpoint]),
            TimerAction(period=17., actions=[frontier_action(ground_parameter_file(frontier_params, prior))], condition=IfCondition(start_exploration)),
        ])]

    actions = [RegisterEventHandler(OnProcessExit(on_exit=physics_process_exited)),
               gazebo, TimerAction(period=5., actions=[mapping])]
    if needs_fresh_ground_alignment(world_path, state, ground_enabled):
        with tempfile.NamedTemporaryFile(prefix='go2_ground_alignment_', suffix='.json', delete=False) as stream:
            alignment_path = stream.name
        resolver = Node(package='fastlio_nav_bridge', executable='ground_prior_resolver',
            name='ground_prior_resolver', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'world': world_path, 'output': alignment_path}])

        def alignment_finished(event, _context):
            return resolved_ground_actions(event, world_path, alignment_path, dependent_actions)

        actions += [
            RegisterEventHandler(OnProcessExit(target_action=resolver, on_exit=alignment_finished)),
            TimerAction(period=8., actions=[resolver]),
        ]
    else:
        prior = continuous_ground_region(world_path, state) if ground_enabled else []
        actions += [TimerAction(period=8., actions=dependent_actions(prior))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='/home/dobby/scan/worlds/research_two_floor.world'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('navigation_rviz', default_value='true'),
        DeclareLaunchArgument('mapping_rviz', default_value='true'),
        DeclareLaunchArgument('record_trial', default_value='true'),
        DeclareLaunchArgument('start_exploration', default_value='true'),
        DeclareLaunchArgument('continuous_ground_floor', default_value='true'),
        DeclareLaunchArgument('resume', default_value='auto'),
        DeclareLaunchArgument('checkpoint_dir', default_value='/home/dobby/scan/checkpoints/current'),
        OpaqueFunction(function=launch_setup),
    ])
