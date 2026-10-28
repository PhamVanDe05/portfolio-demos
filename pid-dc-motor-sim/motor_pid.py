"""DC motor speed control: simulated plant + discrete PID, compare 3 gain sets."""
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Motor parameters (SI): inertia, friction, torque/back-EMF const, resistance, inductance
J, B, K, R, L = 0.002, 0.001, 0.05, 1.0, 0.005
V_MAX = 12.0
DT = 1e-3

GAINS = {
    "P only (Kp=0.5)": (0.5, 0.0, 0.0),
    "Oscillatory (Kp=0.3, Ki=100)": (0.3, 100.0, 0.0),
    "Tuned (Kp=1, Ki=40)": (1.0, 40.0, 0.0),
}


def simulate(kp, ki, kd, setpoint=100.0, t_end=3.0):
    """Return (t, speed[rad/s]). Voltage saturates at +-V_MAX; integrator uses clamping anti-windup."""
    n = int(t_end / DT)
    w = i = integ = prev_err = 0.0
    out = np.empty(n)
    for k in range(n):
        err = setpoint - w
        d = (err - prev_err) / DT if k else 0.0
        u_raw = kp * err + ki * integ + kd * d
        u = float(np.clip(u_raw, -V_MAX, V_MAX))
        if u == u_raw:  # integrate only when not saturated
            integ += err * DT
        prev_err = err
        i += DT * (u - R * i - K * w) / L
        w += DT * (K * i - B * w) / J
        out[k] = w
    return np.arange(n) * DT, out


def metrics(t, y, setpoint=100.0, band=0.02):
    """Overshoot (%) and 2% settling time (s; inf if never settles)."""
    overshoot = max(0.0, (y.max() - setpoint) / setpoint * 100)
    outside = np.where(np.abs(y - setpoint) > band * setpoint)[0]
    if len(outside) == 0:
        return overshoot, 0.0
    if outside[-1] == len(t) - 1:
        return overshoot, float("inf")
    return overshoot, float(t[outside[-1] + 1])


if __name__ == "__main__":
    plt.figure(figsize=(8, 4.5))
    for name, g in GAINS.items():
        t, y = simulate(*g)
        os_, ts = metrics(t, y)
        print(f"{name:28s} overshoot={os_:6.2f}%  settling={ts:.3f}s")
        plt.plot(t, y, label=name)
    plt.axhline(100, color="k", ls="--", lw=0.8)
    plt.xlabel("time [s]"); plt.ylabel("speed [rad/s]")
    plt.title("DC motor step response (setpoint 100 rad/s, 12 V limit)")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    plt.savefig("step_response.png", dpi=120)
