"""Verified first-floor geometry, expressed in the measured mapping frame."""
import math
from .checkpoint_store import world_fingerprint
from .fastlio_tf_bridge import compose_transform, inverse_transform

SLAB_WORLD_SHA256 = '560c02cf29529be83ef5095af29772188fbe5341e3ecca62d893f4628a0f24dd'


def verified_slab_world(world):
    return world_fingerprint(world) == SLAB_WORLD_SHA256


def checked_transform(value):
    position, quaternion = value
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError('Invalid rigid transform dimensions')
    if not all(math.isfinite(v) for v in (*position, *quaternion)):
        raise ValueError('Nonfinite rigid transform')
    if abs(sum(v*v for v in quaternion)-1.) > .001:
        raise ValueError('Rigid transform quaternion is not normalized')
    return tuple(position), tuple(quaternion)


def continuous_ground_region(world, state):
    if state is None or not verified_slab_world(world):
        return []
    transform = compose_transform(checked_transform(state['map_body']),
                                  inverse_transform(checked_transform(state['world_body'])))
    return [float(v) for v in (*transform[0], *transform[1], 9.5, 6.5, .10)]


def needs_fresh_ground_alignment(world, state, enabled=True):
    return enabled and state is None and verified_slab_world(world)


def read_alignment_region(path, world):
    import json
    from pathlib import Path
    data = json.loads(Path(path).read_text())
    if not verified_slab_world(world) or data.get('world_sha256') != SLAB_WORLD_SHA256:
        raise ValueError('Ground alignment belongs to a different world')
    # Recompute from the measured poses rather than trusting an arbitrary region.
    region = continuous_ground_region(world, data)
    if not region:
        raise ValueError('Missing verified ground region')
    return region
