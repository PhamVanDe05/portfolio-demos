import numpy as np
from motor_pid import GAINS, simulate, metrics

TUNED, OSC, P_ONLY = (GAINS[k] for k in
                      ("Tuned (Kp=1, Ki=40)", "Oscillatory (Kp=0.3, Ki=100)", "P only (Kp=0.5)"))


def test_tuned_overshoot_and_settling():
    os_, ts = metrics(*simulate(*TUNED))
    assert os_ < 5.0 and ts < 1.0


def test_p_only_has_steady_state_error():
    _, y = simulate(*P_ONLY)
    assert abs(100 - y[-1]) > 5


def test_tuned_beats_oscillatory():
    assert metrics(*simulate(*TUNED))[1] < metrics(*simulate(*OSC))[1]


def test_voltage_saturation_keeps_output_finite():
    _, y = simulate(1e4, 1e4, 0.0, t_end=0.5)
    assert np.isfinite(y).all()
