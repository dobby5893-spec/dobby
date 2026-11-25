from types import SimpleNamespace
from collections import deque
from unittest.mock import Mock
import struct
import pytest
from fastlio_nav_bridge.exploration_checkpoint import ExplorationCheckpoint
from fastlio_nav_bridge.checkpoint_store import save_checkpoint, load_checkpoint

def test_unverified_resume_cannot_replace_any_committed_generation(tmp_path):
    state = dict(world_sha256='world', map_body=[[0,0,0],[0,0,0,1]],
                 world_body=[[0,0,0],[0,0,0,1]])
    for index in range(3):
        save_checkpoint(tmp_path, state, {'map.ot': str(index).encode()})
    latest = (tmp_path/'LATEST').read_bytes()
    original = {p.name for p in tmp_path.glob('generation-*')}
    node = SimpleNamespace(resume=state, resume_validated=False)
    for _ in range(5):
        with pytest.raises(ValueError, match='until resume alignment is verified'):
            ExplorationCheckpoint.snapshot(node)
    assert (tmp_path/'LATEST').read_bytes() == latest
    assert {p.name for p in tmp_path.glob('generation-*')} == original
    assert load_checkpoint(tmp_path)[1]['world_sha256'] == 'world'

def test_manual_save_reports_deferred_without_touching_saved_progress():
    node = SimpleNamespace(resume={'node_count': 10}, resume_validated=False)
    node.snapshot = lambda: ExplorationCheckpoint.snapshot(node)
    node.state_message = Mock()
    response = SimpleNamespace(success=True, message='')
    result = ExplorationCheckpoint.manual_save(node, None, response)
    assert result.success is False
    assert 'until resume alignment is verified' in result.message
    node.state_message.assert_not_called()

@pytest.mark.parametrize('start_enabled', [True, False])
def test_failed_spawn_validation_never_opens_writer(monkeypatch, start_enabled):
    import fastlio_nav_bridge.exploration_checkpoint as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    body = SimpleNamespace(transform=transform,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    world = SimpleNamespace(position=SimpleNamespace(x=1., y=0., z=0.),
        orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    node = SimpleNamespace(
        managed=False, started=not start_enabled, start_enabled=start_enabled, pending=None,
        recovery_ready_since=None, control=Mock(), map=SimpleNamespace(data=[0]*50),
        floor={'phase': 1}, resume={'node_count': 10, 'map_body': [[0,0,0],[0,0,0,1]],
        'world_body': [[0,0,0],[0,0,0,1]]}, resume_validated=False,
        tf=Mock(), odom=deque([SimpleNamespace(pose=SimpleNamespace(pose=world))]),
        last_log=0., last_save=0., interval=30., state_message=Mock(), get_logger=Mock())
    node.control.service_is_ready.return_value = True
    node.tf.lookup_transform.return_value = body
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000000000))
    node.snapshot = lambda: ExplorationCheckpoint.snapshot(node)
    ExplorationCheckpoint.tick(node)
    assert node.resume_validated is False
    assert node.started is (not start_enabled)
    assert node.last_save == 0.
    node.control.call_async.assert_not_called()
    assert 'startup blocked:' in node.state_message.call_args.args[0]


def test_managed_checkpoint_confirms_only_matching_session(monkeypatch):
    import fastlio_nav_bridge.exploration_checkpoint as module
    session = SimpleNamespace(task_id='current-task', floor_session_id='current-floor', phase=1)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    body = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))
    old_floor = {'phase': 7, 'task_id': session.task_id, 'floor_session_id': 'old-floor'}
    node = SimpleNamespace(
        managed=True, start_enabled=True, session=session, session_seen=99.,
        map=object(), floor=old_floor,
        resume=None, resume_validated=True, recovery_ready_since=96., pending=None,
        tf=Mock(), get_clock=Mock(), get_logger=Mock(), state_message=Mock(),
        session_ready=Mock(), last_log=0., last_save=100., interval=30.)
    node.tf.lookup_transform.return_value = body
    node.get_clock.return_value.now.return_value.nanoseconds = 1000000000
    ExplorationCheckpoint.tick(node)
    node.session_ready.publish.assert_not_called()
    node.floor['floor_session_id'] = session.floor_session_id
    node.recovery_ready_since = 96.
    ExplorationCheckpoint.tick(node)
    node.session_ready.publish.assert_called_once_with(session)
