from fastlio_nav_bridge.resume_alignment import StableAlignment


def transform(z=0):
    return ((0.,0.,z),(0.,0.,0.,1.))


def test_initial_stationary_pose_does_not_bypass_estimator_warmup():
    gate=StableAlignment()
    for t in range(8):
        assert not gate.add(float(t),transform())
    assert gate.add(8.,transform())


def test_centimeter_initialization_drift_keeps_old_map_sealed_until_stable():
    gate=StableAlignment()
    for t in range(12):
        assert not gate.add(float(t),transform(.01*t))
    assert not gate.add(12.,transform(.11))
    assert not gate.add(13.,transform(.11))
    assert gate.add(14.,transform(.11))
