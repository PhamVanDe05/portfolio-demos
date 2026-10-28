import numpy as np
from follow_me import run, rms, limit, V_MAX

W = slice(100, None)


def test_tracking_rms_below_threshold():
    r = run()
    assert rms(r["drone"][W], r["truth"][W]) < 1.2


def test_kalman_beats_raw_measurements():
    r = run()
    assert rms(r["est"][W], r["truth"][W]) < 0.7 * rms(r["meas"][W], r["truth"][W])


def test_speed_limit():
    assert np.linalg.norm(limit(np.array([30.0, 40.0]))) <= V_MAX + 1e-9
    assert np.allclose(limit(np.array([1.0, 0.0])), [1.0, 0.0])
