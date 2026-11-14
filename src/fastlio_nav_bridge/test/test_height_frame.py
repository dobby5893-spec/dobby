import math
from fastlio_nav_bridge.fastlio_tf_bridge import compose_transform, inverse_transform


def test_foot_height_is_subtracted_once_and_reconstruction_matches_body():
    stance = ((0.0, 0.0, 0.208), (0.0, 0.0, 0.0, 1.0))
    body = ((3.1, -0.73, 0.136), (0.0, 0.0, 0.0, 1.0))
    foot = compose_transform(body, inverse_transform(stance))
    assert abs(foot[0][2] + 0.072) < 1e-10
    restored = compose_transform(foot, stance)
    assert all(abs(a-b) < 1e-10 for a, b in zip(restored[0], body[0]))


def test_composition_preserves_body_in_climb_with_pitch_and_changed_stance():
    pitch = 0.2
    body = ((2.0, 1.0, 1.3), (0.0, math.sin(pitch/2), 0.0, math.cos(pitch/2)))
    stance = ((0.01, 0.0, 0.24), (0.0, 0.0, 0.0, 1.0))
    foot = compose_transform(body, inverse_transform(stance))
    restored = compose_transform(foot, stance)
    for actual, expected in zip(restored[0]+restored[1], body[0]+body[1]):
        assert abs(actual-expected) < 1e-10
    assert foot[0][2] > 1.0
