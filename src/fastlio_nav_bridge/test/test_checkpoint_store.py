import json
import struct

import pytest
from fastlio_nav_bridge.checkpoint_store import full_octomap_file, save_checkpoint, load_checkpoint


def state(world='world-a'):
    return dict(world_sha256=world, map_body=[[1.,2.,.2],[0.,0.,0.,1.]],
                world_body=[[1.,2.,.2],[0.,0.,0.,1.]])


def test_full_map_preserves_free_probability_and_tree_structure():
    raw = struct.pack('<fBfBfB', 0.3, 3, -1.7, 0, 2.1, 0)
    data = full_octomap_file(raw, .05)
    assert b'size 3\n' in data
    assert data.split(b'data\n', 1)[1] == raw
    with pytest.raises(ValueError):
        full_octomap_file(raw[:-1], .05)
    with pytest.raises(ValueError):
        full_octomap_file(raw[:5], .05)


def test_restart_loads_last_committed_generation_and_rejects_wrong_world(tmp_path):
    directory = save_checkpoint(tmp_path, state(), {'map.ot': b'fullmap'})
    loaded, metadata = load_checkpoint(tmp_path, 'world-a')
    assert loaded == directory
    assert metadata['map_body'][0] == [1.,2.,.2]
    with pytest.raises(ValueError, match='different world'):
        load_checkpoint(tmp_path, 'world-b')


def test_interrupted_save_and_corrupt_latest_fall_back_to_valid_map(tmp_path):
    previous = save_checkpoint(tmp_path, state(), {'map.ot': b'previous'})
    current = save_checkpoint(tmp_path, state(), {'map.ot': b'current'})
    (tmp_path/'.generation-interrupted').mkdir()
    (current/'map.ot').write_bytes(b'corrupted')
    assert load_checkpoint(tmp_path)[0] == previous


def test_retention_is_bounded_and_keeps_two_rollback_copies(tmp_path):
    for i in range(5):
        save_checkpoint(tmp_path, state(), {'map.ot': str(i).encode()})
    assert len(list(tmp_path.glob('generation-*'))) == 3
    assert (load_checkpoint(tmp_path)[0]/'map.ot').read_bytes() == b'4'


def test_invalid_pose_and_traversal_are_rejected(tmp_path):
    bad = state()
    bad['map_body'][1] = [0,0,0,0]
    save_checkpoint(tmp_path, bad, {'map.ot': b'bad'})
    with pytest.raises(ValueError, match='quaternion'):
        load_checkpoint(tmp_path)
    with pytest.raises(ValueError):
        save_checkpoint(tmp_path, state(), {'../outside': b'bad'})
