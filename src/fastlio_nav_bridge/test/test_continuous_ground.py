"""Regression checks for fresh and resumed verified floor priors."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from fastlio_nav_bridge.continuous_ground import (
    SLAB_WORLD_SHA256, continuous_ground_region, needs_fresh_ground_alignment,
    read_alignment_region)
from fastlio_nav_bridge.checkpoint_store import save_checkpoint

ROOT = Path(__file__).resolve().parents[3]
WORLD = ROOT / 'worlds/research_two_floor.world'


def launch_module():
    path = ROOT / 'src/go2_multifloor_exploration/launch/full_multifloor_demo.launch.py'
    spec = importlib.util.spec_from_file_location('ground_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample_state():
    return dict(world_sha256=SLAB_WORLD_SHA256,
                map_body=[[1., 2., .25], [0., 0., 0., 1.]],
                world_body=[[0., 0., .22], [0., 0., 0., 1.]])


def test_fresh_known_world_requires_alignment_before_navigation():
    assert needs_fresh_ground_alignment(WORLD, None)
    assert not needs_fresh_ground_alignment(WORLD, sample_state())
    assert not needs_fresh_ground_alignment(WORLD, None, False)


def test_unknown_scene_cannot_inherit_solid_ground(tmp_path):
    world = tmp_path / 'other.world'
    world.write_text('Different scene with possible holes')
    assert not needs_fresh_ground_alignment(world, None)
    assert continuous_ground_region(world, sample_state()) == []


def test_resume_region_uses_measured_world_to_map_transform():
    region = continuous_ground_region(WORLD, sample_state())
    assert region[:3] == pytest.approx([1., 2., .03])
    assert region[3:7] == pytest.approx([0., 0., 0., 1.])
    assert region[7:] == [9.5, 6.5, .1]


def test_rotation_is_preserved_instead_of_assuming_world_equals_map():
    state = sample_state()
    state['map_body'] = [[1., 2., .25], [0., 0., math.sin(.4), math.cos(.4)]]
    state['world_body'][0][:2] = [2., 1.]
    region = continuous_ground_region(WORLD, state)
    assert region[:2] == pytest.approx([
        1.-2.*math.cos(.8)+math.sin(.8),
        2.-2.*math.sin(.8)-math.cos(.8)])
    assert region[3:7] == pytest.approx(state['map_body'][1])


@pytest.mark.parametrize('bad', [
    [[0., 0., float('nan')], [0., 0., 0., 1.]],
    [[0., 0., 0.], [0., 0., 0., 0.]],
    [[0., 0.], [0., 0., 0., 1.]],
])
def test_invalid_alignment_is_not_a_floor_prior(bad):
    state = sample_state()
    state['map_body'] = bad
    with pytest.raises(ValueError):
        continuous_ground_region(WORLD, state)


def test_completed_alignment_is_recomputed_and_bound_to_world(tmp_path):
    path = tmp_path / 'alignment.json'
    state = sample_state()
    state['region'] = [999.] * 10
    path.write_text(json.dumps(state))
    assert read_alignment_region(path, WORLD) == continuous_ground_region(WORLD, state)
    state['world_sha256'] = 'wrong'
    path.write_text(json.dumps(state))
    with pytest.raises(ValueError, match='different world'):
        read_alignment_region(path, WORLD)


@pytest.mark.parametrize('mode', ['fresh', 'resume', 'disabled'])
def test_launch_gates_all_ground_consumers_until_fresh_alignment(tmp_path, mode):
    from launch import LaunchContext
    from launch.actions import RegisterEventHandler
    module = launch_module()
    if mode == 'resume':
        save_checkpoint(tmp_path, sample_state(), {'map.ot': b'fixture'})
    context = LaunchContext()
    context.launch_configurations.update({
        'use_sim_time': 'true', 'world': str(WORLD),
        'gazebo_gui': 'false', 'navigation_rviz': 'false', 'mapping_rviz': 'false',
        'record_trial': 'false', 'start_exploration': 'true',
        'checkpoint_dir': str(tmp_path), 'resume': 'true' if mode == 'resume' else 'false',
        'continuous_ground_floor': 'false' if mode == 'disabled' else 'true',
    })
    actions = module.launch_setup(context)
    handlers = [a for a in actions if isinstance(a, RegisterEventHandler)]
    assert len(handlers) == (1 if mode == 'fresh' else 0)
    assert len(actions) == (4 if mode == 'fresh' else 3)


def test_failed_resolver_cannot_launch_navigation(tmp_path):
    module = launch_module()
    calls = []
    with pytest.raises(RuntimeError, match='not started'):
        module.resolved_ground_actions(
            SimpleNamespace(returncode=1), WORLD, tmp_path/'absent.json',
            lambda region: calls.append(region))
    assert calls == []


def test_successful_resolver_supplies_same_prior_to_dependent_group(tmp_path):
    module = launch_module()
    path = tmp_path/'aligned.json'
    state = sample_state()
    path.write_text(json.dumps(state))
    calls = []
    result = module.resolved_ground_actions(SimpleNamespace(returncode=0), WORLD, path,
        lambda region: calls.append(region) or ['navigation_group'])
    assert result == ['navigation_group']
    assert calls == [continuous_ground_region(WORLD, state)]
