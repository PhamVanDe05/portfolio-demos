"""Follow-me drone sim: noisy 2D target -> constant-velocity Kalman filter -> P velocity controller."""
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DT = 0.05
MEAS_SIGMA = 1.0   # [m] position measurement noise
V_MAX = 4.0        # [m/s] drone speed limit
KP = 1.5           # [1/s] position-error gain


class KalmanCV:
    """Constant-velocity Kalman filter, state [x, y, vx, vy], position-only measurements."""

    def __init__(self, z0, dt=DT, sigma=MEAS_SIGMA, q=1.0):
        self.x = np.array([z0[0], z0[1], 0.0, 0.0])
        self.P = np.diag([sigma**2, sigma**2, 4.0, 4.0])
        self.F = np.eye(4); self.F[0, 2] = self.F[1, 3] = dt
        self.H = np.zeros((2, 4)); self.H[0, 0] = self.H[1, 1] = 1.0
        self.R = np.eye(2) * sigma**2
        g = np.array([[dt**2 / 2], [dt]])  # white-noise acceleration, per axis
        Q1 = q * g @ g.T
        self.Q = np.zeros((4, 4))
        for a in (0, 1):
            self.Q[np.ix_([a, a + 2], [a, a + 2])] = Q1

    def step(self, z):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        S = self.H @ self.P @ self.H.T + self.R
        Kg = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + Kg @ (np.asarray(z) - self.H @ self.x)
        self.P = (np.eye(4) - Kg @ self.H) @ self.P
        return self.x.copy()


def limit(v, vmax=V_MAX):
    n = np.linalg.norm(v)
    return v if n <= vmax else v * vmax / n


def run(steps=600, seed=0):
    """Return dict of arrays: truth, meas, est (target position) and drone position."""
    rng = np.random.default_rng(seed)
    t = np.arange(steps) * DT
    truth = np.stack([8 * np.cos(0.15 * t), 5 * np.sin(0.3 * t)], axis=1)  # ~1-1.5 m/s
    meas = truth + rng.normal(0, MEAS_SIGMA, truth.shape)
    kf = KalmanCV(meas[0])
    drone = np.array([-5.0, -5.0])
    est, drn = np.empty_like(truth), np.empty_like(truth)
    for k in range(steps):
        s = kf.step(meas[k])
        v_cmd = limit(KP * (s[:2] - drone) + s[2:])  # P on position + estimated target velocity
        drone = drone + v_cmd * DT
        est[k], drn[k] = s[:2], drone
    return dict(truth=truth, meas=meas, est=est, drone=drn)


def rms(a, b):
    return float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))


if __name__ == "__main__":
    r = run()
    w = slice(100, None)  # skip the initial approach
    print(f"raw measurement RMS error : {rms(r['meas'][w], r['truth'][w]):.2f} m")
    print(f"Kalman estimate RMS error : {rms(r['est'][w], r['truth'][w]):.2f} m")
    print(f"drone tracking RMS error  : {rms(r['drone'][w], r['truth'][w]):.2f} m")
    plt.figure(figsize=(6.5, 5.5))
    plt.plot(*r["meas"].T, ".", ms=2, c="lightgray", label="noisy measurements")
    plt.plot(*r["truth"].T, "k--", label="target (truth)")
    plt.plot(*r["est"].T, "b-", lw=1, label="Kalman estimate")
    plt.plot(*r["drone"].T, "r-", lw=1.5, label="drone")
    plt.axis("equal"); plt.grid(alpha=0.3); plt.legend(loc="upper right", fontsize=8)
    plt.xlabel("x [m]"); plt.ylabel("y [m]"); plt.title("Follow-me with Kalman filter")
    plt.tight_layout(); plt.savefig("trajectory.png", dpi=120)
