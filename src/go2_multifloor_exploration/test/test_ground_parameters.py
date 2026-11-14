
import importlib.util
import json
from pathlib import Path
import rclpy

def launch_module():
    path=Path(__file__).resolve().parents[1]/'launch/full_multifloor_demo.launch.py'
    spec=importlib.util.spec_from_file_location('ground_launch_test',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def test_explicit_ground_arrays_reach_both_included_node_configs(tmp_path):
    module=launch_module()
    prior=[0.,0.,-.21,0.,0.,0.,1.,9.5,6.5,.1]
    source=tmp_path/'input.yaml'
    source.write_text(json.dumps({
        'controller_server':{'ros__parameters':{'use_sim_time':True}},
        '/frontier_explorer':{'ros__parameters':{'use_sim_time':True}}}))
    merged=module.ground_parameter_file(str(source),prior)
    rclpy.init()
    try:
        for name in ['controller_server','frontier_explorer']:
            node=rclpy.create_node(name,
                automatically_declare_parameters_from_overrides=True,
                cli_args=['--ros-args','--params-file',merged])
            try:
                assert list(node.get_parameter('continuous_floor_region').value)==prior
            finally:
                node.destroy_node()
    finally:
        rclpy.shutdown()
        Path(merged).unlink()

def test_disabled_ground_prior_does_not_invent_a_floor(tmp_path):
    module=launch_module()
    source=tmp_path/'params.yaml'
    source.write_text('{}')
    assert module.ground_parameter_file(str(source),[])==str(source)

def test_physics_exit_stops_stack_but_pause_and_other_processes_do_not():
    from types import SimpleNamespace
    from launch.actions import EmitEvent
    module=launch_module()
    event=SimpleNamespace(cmd=['/usr/bin/gzserver'], returncode=-11)
    actions=module.physics_process_exited(event,SimpleNamespace(is_shutdown=False))
    assert len(actions)==2 and isinstance(actions[-1],EmitEvent)
    event.cmd=['rviz2']
    assert module.physics_process_exited(event,SimpleNamespace(is_shutdown=False))==[]
    event.cmd=['gzserver']
    assert module.physics_process_exited(event,SimpleNamespace(is_shutdown=True))==[]
