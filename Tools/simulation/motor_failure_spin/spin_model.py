#!/usr/bin/env python3
"""
Quadrotor with one motor dead: equilibria and a spinning reduced-attitude controller.

Dynamics copy the SIH quadrotor model (src/modules/simulation/simulator_sih/sih.cpp):
linear thrust in the motor command u in [0, 1], first-order motor lag SIH_T_TAU,
isotropic angular damping SIH_KDW, linear translational drag SIH_KDV, FRD body frame, NED world.

Motor layout (PX4 numbering, SIH):  m0 (+x,+y) CCW   m1 (-x,-y) CCW
                                    m2 (+x,-y) CW    m3 (-x,+y) CW
Motor 0 is the one that fails. Motor 1 is diagonally opposite.
"""
import argparse
import numpy as np

G = 9.81


class Vehicle:
    def __init__(self, mass=1.0, ixx=0.025, izz=0.03, t_max=5.0, q_max=0.1, arm=0.2, kdw=0.025, kdv=1.0,
                 tau=0.05, w_max=6 * np.pi):
        self.m, self.ix, self.iz = mass, ixx, izz
        self.I = np.diag([ixx, ixx, izz])
        self.Iinv = np.linalg.inv(self.I)
        self.t_max, self.q_max, self.L = t_max, q_max, arm
        self.kappa = q_max / t_max          # yaw torque per newton of thrust [m]
        self.kdw, self.kdv, self.tau, self.w_max = kdw, kdv, tau, w_max

    def wrench(self, f):
        """Body torque and thrust from the four motor forces [N]."""
        L, k = self.L, self.kappa
        tx = L * (-f[0] + f[1] + f[2] - f[3])
        ty = L * (f[0] - f[1] + f[2] - f[3])
        tz = k * (f[0] + f[1] - f[2] - f[3])
        return np.array([tx, ty, tz]), f.sum()

    def alloc3(self, tx, ty, thrust):
        """Exact allocation with motor 0 removed and yaw dropped: (tx, ty, T) -> f1, f2, f3."""
        L = self.L
        # tx = L(f1 + f2 - f3), ty = L(-f1 + f2 - f3), T = f1 + f2 + f3
        A = np.array([[L, L, -L], [-L, L, -L], [1, 1, 1]])
        return np.linalg.solve(A, [tx, ty, thrust])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def equilibrium(v, f1):
    """
    Relaxed-hover equilibrium with motor 0 dead, f2 == f3 and opposite motor thrust f1.
    Fixed point on the spin rate: r from the yaw balance, body tilt from the gyroscopic balance.
    Returns dict or None when it does not exist.
    """
    D, k, L = v.iz - v.ix, v.kdw, v.L
    c = np.sqrt(2) * L * f1                 # torque about the m2-m3 diagonal that f1 makes
    T = v.m * G
    r = -5.0

    for _ in range(200):
        den = (D * r) ** 2 + k ** 2
        pa, pb = D * r * c / den, k * c / den  # body rates along/perpendicular to the m0-m1 diagonal
        p, q = (pa + pb) / np.sqrt(2), (pa - pb) / np.sqrt(2)
        w = np.array([p, q, r])
        cos_tilt = abs(r) / np.linalg.norm(w)
        T = v.m * G / cos_tilt
        f23 = (T - f1) / 2
        r_new = v.kappa * (f1 - 2 * f23) / k
        if abs(r_new - r) < 1e-9:
            break
        r = 0.5 * r + 0.5 * r_new

    f = np.array([0.0, f1, f23, f23])
    if f23 < 0 or f.max() > v.t_max:
        return None
    return dict(w=w, f=f, u=f / v.t_max, tilt_deg=np.degrees(np.arccos(cos_tilt)), rps=abs(r) / (2 * np.pi),
                n=w / np.linalg.norm(w))


class SpinController:
    """
    Reduced-attitude control while spinning freely about yaw.

    Outer loop: align the body-fixed axis n (the equilibrium spin axis) with the desired thrust direction.
    Inner loop: dynamic inversion on body roll/pitch rates including the gyroscopic term.
    Yaw is not controlled. The opposite motor's thrust is set through the axis n (tilt grows the
    gyroscopic moment the opposite motor has to balance).
    """

    def __init__(self, v, f1_ratio=0.5, tilt_max_deg=25, k_att=4.0, k_rate=12.0, use_model=True):
        self.v, self.f1_ratio, self.tilt_max = v, f1_ratio, np.radians(tilt_max_deg)
        self.k_att, self.k_rate, self.use_model = k_att, k_rate, use_model

    def spin_axis(self, r, thrust):
        """Body axis that, spinning at r, needs exactly the torque the opposite motor makes at f1."""
        v = self.v
        D, k, L = v.iz - v.ix, v.kdw, v.L
        f1 = self.f1_ratio * thrust / (2 + self.f1_ratio)
        c = np.sqrt(2) * L * f1
        den = (D * r) ** 2 + k ** 2
        pa, pb = D * r * c / den, k * c / den
        pxy = np.array([(pa + pb) / np.sqrt(2), (pa - pb) / np.sqrt(2)])
        # limit the tilt; while r is still small this just means the opposite motor runs lower
        lim = abs(r) * np.tan(self.tilt_max)
        if np.linalg.norm(pxy) > lim:
            pxy *= lim / np.linalg.norm(pxy)
        n = np.array([pxy[0], pxy[1], r if abs(r) > 1e-3 else -1e-3])
        n /= np.linalg.norm(n)
        return n if n[2] < 0 else -n

    def __call__(self, R, w, thrust_dir_W, thrust_mag):
        v = self.v
        h = R.T @ thrust_dir_W              # desired thrust direction in body frame
        # thrust acts along -z_B; while spinning we use the body axis n instead
        n = self.spin_axis(w[2], v.m * G)
        e = np.cross(n, h)                  # rotate n towards h
        # spin about h (keeps it still in body frame) plus a correction; z is uncontrolled
        lam = (w[2] - self.k_att * e[2]) / h[2] if abs(h[2]) > 0.2 else 0.0
        w_sp = lam * h + self.k_att * e
        w_sp = np.array([w_sp[0], w_sp[1]])

        # inner loop
        acc = self.k_rate * (w_sp - w[:2])
        tau = v.ix * acc
        if self.use_model:
            gyro = np.cross(w, v.I @ w) + v.kdw * w
            tau = tau + gyro[:2]

        # thrust along the desired direction: -z_B projected on h
        cos_t = max(-h[2], 0.5)
        T = thrust_mag / cos_t
        f123 = v.alloc3(tau[0], tau[1], T)
        f = np.concatenate([[0.0], f123])
        return np.clip(f / v.t_max, 0.0, 1.0)


def dlqr(A, B, Q, R, dt, iterations=30):
    """LQR gain for the continuous model sampled at dt, DARE solved by structure-preserving doubling."""
    n = A.shape[0]
    Ad = np.eye(n) + A * dt + A @ A * dt * dt / 2
    Bd = B * dt
    Qd, Rd = Q * dt, R * dt
    Ak, Gk, Hk = Ad, Bd @ np.linalg.solve(Rd, Bd.T), Qd
    for _ in range(iterations):
        W = np.linalg.inv(np.eye(n) + Gk @ Hk)
        Ak, Gk, Hk = Ak @ W @ Ak, Gk + Ak @ W @ Gk @ Ak.T, Hk + Ak.T @ Hk @ W @ Ak
    P = Hk
    return np.linalg.solve(Rd + Bd.T @ P @ Bd, Bd.T @ P @ Ad)


class LqrSpinController(SpinController):
    """
    Same equilibrium as SpinController, but the roll/pitch loop is an LQR on the dynamics linearised
    about the spinning equilibrium (body frame, yaw rate frozen), including the motor lag.
    State: [p, q, hx, hy, tau_x, tau_y], h = desired thrust direction in body frame.
    Gains are recomputed as the spin rate changes (gain scheduling on r).
    """

    def __init__(self, v, q_diag=(1, 1, 400, 400, 0, 0), r_diag=(400, 400), **kw):
        super().__init__(v, **kw)
        self.Q, self.R = np.diag(q_diag), np.diag(r_diag)
        self.tau_est = np.zeros(2)
        self.K, self.K_r = None, None
        self.dt = 0.001
        self.dt_ctrl = 0.004

    def _model(self, x, tau_cmd, r):
        v = self.v
        p, q, hx, hy, tx, ty = x
        hz = -np.sqrt(max(1 - hx * hx - hy * hy, 1e-6))
        w = np.array([p, q, r])
        h = np.array([hx, hy, hz])
        w_dot = v.Iinv @ (np.array([tx, ty, 0]) - v.kdw * w - np.cross(w, v.I @ w))
        h_dot = -np.cross(w, h)
        t_dot = (np.asarray(tau_cmd) - [tx, ty]) / v.tau
        return np.array([w_dot[0], w_dot[1], h_dot[0], h_dot[1], t_dot[0], t_dot[1]])

    def linearise(self, r):
        v = self.v
        n = self.spin_axis(r, v.m * G)
        w = r / n[2] * n
        tau = np.cross(w, v.I @ w) + v.kdw * w
        x0 = np.array([w[0], w[1], n[0], n[1], tau[0], tau[1]])
        u0 = tau[:2]
        eps = 1e-5
        A = np.zeros((6, 6))
        B = np.zeros((6, 2))
        for i in range(6):
            dx = np.zeros(6)
            dx[i] = eps
            A[:, i] = (self._model(x0 + dx, u0, r) - self._model(x0 - dx, u0, r)) / (2 * eps)
        for i in range(2):
            du = np.zeros(2)
            du[i] = eps
            B[:, i] = (self._model(x0, u0 + du, r) - self._model(x0, u0 - du, r)) / (2 * eps)
        K = dlqr(A, B, self.Q, self.R, self.dt_ctrl)
        return x0, u0, K, A, B

    def __call__(self, R, w, thrust_dir_W, thrust_mag):
        v = self.v
        r = w[2]
        if self.K is None or abs(r - self.K_r) > 0.25:
            self.x0, self.u0, self.K, _, _ = self.linearise(r)
            self.K_r = r
        h = R.T @ thrust_dir_W
        x = np.array([w[0], w[1], h[0], h[1], self.tau_est[0], self.tau_est[1]])
        tau_cmd = self.u0 - self.K @ (x - self.x0)
        cos_t = max(-h[2], 0.5)
        T = thrust_mag / cos_t
        f = np.concatenate([[0.0], v.alloc3(tau_cmd[0], tau_cmd[1], T)])
        u = np.clip(f / v.t_max, 0.0, 1.0)
        # what the motors will actually produce (saturated), passed through the motor lag model
        tau_real, _ = v.wrench(u * v.t_max)
        self.tau_est += self.dt / v.tau * (tau_real[:2] - self.tau_est)
        return u


def simulate(v, ctrl, t_end=20.0, dt=0.001, t_fail=1.0, pos_sp=np.zeros(3), log_every=10, kp=1.0, kv=2.0,
             delay=0.0, detect=0.0, thrust_cap=None):
    """
    Hover on a simple cascaded PD position loop; motor 0 dies at t_fail.
    delay: transport delay from controller output to the motors [s]
    detect: time until the failure is detected; until then the nominal controller runs on [s]
    thrust_cap: limit on the total thrust command in spin mode [N]
    """
    from collections import deque
    buf = deque([None] * max(int(round(delay / dt)), 0))
    q = np.array([1.0, 0, 0, 0])
    w = np.zeros(3)
    pos, vel = np.zeros(3), np.zeros(3)
    u = np.full(4, v.m * G / 4 / v.t_max)
    log = []

    for i in range(int(t_end / dt)):
        t = i * dt
        R = quat_to_R(q)
        failed = t >= t_fail

        # position loop -> desired thrust vector (NED, points along the force we want)
        a_sp = kp * (pos_sp - pos) + kv * (-vel)
        f_sp = v.m * (a_sp - np.array([0, 0, G])) + v.kdv * vel * 0
        tilt_lim = np.radians(20)
        horiz = np.linalg.norm(f_sp[:2])
        if horiz > -f_sp[2] * np.tan(tilt_lim):
            f_sp[:2] *= -f_sp[2] * np.tan(tilt_lim) / horiz
        d = f_sp / np.linalg.norm(f_sp)

        if t >= t_fail + detect:
            T_cmd = -f_sp[2] if thrust_cap is None else min(-f_sp[2], thrust_cap)
            u_sp = ctrl(R, w, d, T_cmd)
        else:
            u_sp = nominal(v, R, w, d, np.linalg.norm(f_sp))

        buf.append(u_sp)
        u_del = buf.popleft()
        if u_del is None:
            u_del = u_sp if delay == 0 else np.full(4, v.m * G / 4 / v.t_max)
        u = u + dt / v.tau * (u_del - u)
        if failed:
            u[0] = 0.0
        f = u * v.t_max
        tau, T = v.wrench(f)
        tau = tau - v.kdw * w
        w_dot = v.Iinv @ (tau - np.cross(w, v.I @ w))
        w = np.clip(w + w_dot * dt, -v.w_max, v.w_max)
        dq = np.concatenate([[0.0], w])
        q = q + 0.5 * quat_mul(q, dq) * dt
        q /= np.linalg.norm(q)
        acc = R @ np.array([0, 0, -T]) / v.m + np.array([0, 0, G]) - v.kdv * vel / v.m
        vel = vel + acc * dt
        pos = pos + vel * dt

        if i % log_every == 0:
            log.append(np.concatenate([[t], pos, vel, w, u, R[:, 2]]))

        if pos[2] > 20 or np.any(~np.isfinite(pos)):
            break

    return np.array(log)


def nominal(v, R, w, d, T):
    """Plain attitude + rate PD with the full X mixer (stands in for PX4 before the failure)."""
    z_des = -d
    e = np.cross(R[:, 2], z_des)
    e_B = R.T @ e
    w_sp = 6.0 * e_B
    tau = v.ix * 15 * (w_sp - w) + np.cross(w, v.I @ w)
    tau[2] = v.iz * 5 * (0 - w[2])
    L, k = v.L, v.kappa
    B = np.array([[-L, L, L, -L], [L, -L, L, -L], [k, k, -k, -k], [1, 1, 1, 1]])
    f = np.linalg.solve(B, [tau[0], tau[1], tau[2], T])
    return np.clip(f / v.t_max, 0, 1)


VEHICLES = {
    'sih_default': dict(),
    # plausible 1 kg X quad: planar mass distribution (Izz ~ 2 Ixx), little rotational drag
    'realistic': dict(ixx=0.010, izz=0.018, kdw=0.005, t_max=6.0, q_max=0.12),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vehicle', default='realistic', choices=VEHICLES)
    ap.add_argument('--f1', type=float, default=0.5, help='opposite motor thrust / adjacent motor thrust')
    ap.add_argument('--tau', type=float, default=None)
    ap.add_argument('--plot', default=None)
    ap.add_argument('--ctrl', default='lqr', choices=['lqr', 'cascade'])
    ap.add_argument('--kdw', type=float, default=None)
    a = ap.parse_args()

    kw = dict(VEHICLES[a.vehicle])
    if a.tau:
        kw['tau'] = a.tau
    if a.kdw:
        kw['kdw'] = a.kdw
    v = Vehicle(**kw)

    print(f'{a.vehicle}: hover u (4 motors) {v.m * G / 4 / v.t_max:.2f}, T/W {4 * v.t_max / v.m / G:.2f}')
    print(' f1[N]  ratio  rps   tilt   u1    u2=u3')
    for f1 in np.linspace(0, 5, 11):
        eq = equilibrium(v, f1)
        if eq:
            print(f'{f1:5.2f}  {f1 / eq["f"][2]:5.2f} {eq["rps"]:5.2f} {eq["tilt_deg"]:5.1f}  '
                  f'{eq["u"][1]:.2f}  {eq["u"][2]:.2f}')

    ctrl = (LqrSpinController if a.ctrl == 'lqr' else SpinController)(v, f1_ratio=a.f1)
    log = simulate(v, ctrl)
    t, pos, w, u = log[:, 0], log[:, 1:4], log[:, 7:10], log[:, 10:14]
    late = t > t[-1] - 5
    print(f'sim end t={t[-1]:.1f}s pos={pos[-1].round(2)} max|pos| last 5s={np.abs(pos[late]).max(axis=0).round(2)} '
          f'r={w[-1, 2]:.1f} rad/s u={u[-1].round(2)}')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
        ax[0].plot(t, pos)
        ax[0].legend(['x', 'y', 'z'])
        ax[0].set_ylabel('pos [m]')
        ax[1].plot(t, w)
        ax[1].legend(['p', 'q', 'r'])
        ax[1].set_ylabel('rate [rad/s]')
        ax[2].plot(t, u)
        ax[2].legend(['u0', 'u1', 'u2', 'u3'])
        ax[3].plot(t, np.degrees(np.arccos(np.clip(-log[:, 16], -1, 1))))
        ax[3].set_ylabel('body z tilt [deg]')
        fig.savefig(a.plot, dpi=90)


if __name__ == '__main__':
    main()
