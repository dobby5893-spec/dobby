"""Durable simulation checkpoints. No ROS objects or old timestamps are replayed."""
import hashlib
import json
import math
import os
import shutil
from pathlib import Path
import struct
import sys
import time
import uuid


def digest(data):
    return hashlib.sha256(data).hexdigest()


def world_fingerprint(path):
    return digest(Path(path).read_bytes())


def full_octomap_file(data, resolution):
    """OcTree full format: float log odds + child mask, recursively, per node."""
    if sys.byteorder != 'little' or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Unsupported OctoMap platform/resolution')
    if not data or len(data) % 5:
        raise ValueError('Truncated full OcTree')
    stack = [0]
    offset = 0
    while stack:
        depth = stack.pop()
        if offset + 5 > len(data) or depth > 16:
            raise ValueError('Invalid OcTree structure')
        value, children = struct.unpack_from('<fB', data, offset)
        if not math.isfinite(value):
            raise ValueError('Invalid OcTree log odds')
        offset += 5
        stack.extend([depth + 1] * children.bit_count())
    if offset != len(data):
        raise ValueError('Trailing OcTree data')
    header = ('# Octomap OcTree file\nid OcTree\nsize %d\nres %.17g\ndata\n'
              % (len(data) // 5, resolution)).encode('ascii')
    return header + data


def _write(path, data):
    with path.open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def _sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def save_checkpoint(root, state, files):
    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    name = 'generation-%d-%s' % (time.time_ns(), uuid.uuid4().hex[:8])
    staging = root / ('.' + name)
    staging.mkdir()
    for filename, data in files.items():
        if Path(filename).name != filename:
            raise ValueError('Checkpoint file must be a basename')
        _write(staging / filename, data)
    manifest = dict(state, schema=1,
                    files={k: digest(v) for k, v in files.items()})
    _write(staging / 'state.json', json.dumps(manifest, allow_nan=False,
                                             indent=2).encode())
    _sync_dir(staging)
    os.replace(staging, root / name)
    _sync_dir(root)
    pointer = root / ('.LATEST-' + uuid.uuid4().hex)
    _write(pointer, (name + '\n').encode())
    os.replace(pointer, root / 'LATEST')
    _sync_dir(root)
    # Keep the last three committed generations, including two rollback copies.
    for old in sorted(root.glob('generation-*'), reverse=True)[3:]:
        if old.is_dir() and not old.is_symlink() and old.parent.resolve() == root:
            shutil.rmtree(old)
    return root / name


def load_checkpoint(root, expected_world=None):
    root = Path(root).expanduser().resolve()
    preferred = (root / 'LATEST').read_text().strip() if (root / 'LATEST').exists() else ''
    names = sorted((p.name for p in root.glob('generation-*') if p.is_dir()), reverse=True)
    if preferred in names:
        names.remove(preferred)
        names.insert(0, preferred)
    errors = []
    for name in names:
        try:
            directory = root / name
            state = json.loads((directory / 'state.json').read_text())
            if state.get('schema') != 1 or 'map.ot' not in state.get('files', {}):
                raise ValueError('Unsupported checkpoint format')
            for filename, checksum in state['files'].items():
                if Path(filename).name != filename:
                    raise ValueError('Invalid file path')
                if digest((directory / filename).read_bytes()) != checksum:
                    raise ValueError('Checksum mismatch: ' + filename)
            if expected_world is not None and state['world_sha256'] != expected_world:
                raise ValueError('Checkpoint belongs to a different world')
            for key in ('map_body', 'world_body'):
                p, q = state[key]
                if len(p) != 3 or len(q) != 4 or not all(math.isfinite(x) for x in p + q):
                    raise ValueError('Invalid saved pose')
                if abs(sum(v*v for v in q) - 1) > 0.01:
                    raise ValueError('Invalid saved quaternion')
            return directory, state
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append(name + ': ' + str(error))
    raise ValueError('No valid checkpoint: ' + '; '.join(errors[-3:]))
