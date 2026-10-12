#!/usr/bin/env python3
"""Python-model sweeps: motor time constant vs spin rate, and which vehicles can hold a hover at all."""
import numpy as np
import spin_model as s

CAP = 0.9 * 2.5


def holds(v, ctrl_kw=None, delay=0.005, detect=0.0, t_end=25):
    """True when it neither crashes nor keeps oscillating; also the spin rate and the worst altitude loss."""
    c = s.LqrSpinController(v, f1_ratio=0.5, **(ctrl_kw or {}))
    log = s.simulate(v, c, t_end=t_end, delay=delay, detect=detect, thrust_cap=CAP * v.t_max)
    t, pos, w, u = log[:, 0], log[:, 1:4], log[:, 7:10], log[:, 10:14]
    if t[-1] < t_end - 0.1:
        return False, w[-1, 2], pos[:, 2].max()
    late = t > t_end - 4
    ok = np.abs(pos[late]).max() < 1.0 and np.ptp(u[late, 1:], axis=0).max() < 0.1
    return ok, np.mean(w[late, 2]), pos[:, 2].max()


def main():
    print('motor time constant vs spin rate (realistic inertia, rate limit 40 rad/s)')
    print('KDW     spin[rad/s]  max tau that still holds [ms]')
    for kdw in [0.012, 0.008, 0.005, 0.0035]:
        best, rspin, loss = None, None, None
        for tau in [0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.15, 0.2, 0.25, 0.3]:
            v = s.Vehicle(**dict(s.VEHICLES['realistic'], kdw=kdw, tau=tau, w_max=40))
            ok, r, l = holds(v)
            if ok:
                best, rspin, loss = tau, r, l
            else:
                break
        print('%.4f  %6.1f       %s   (alt loss at that tau %.1f m)' % (kdw, rspin if rspin else float('nan'),
                                                                       int(best * 1000) if best else '-', loss or 0))

    print()
    print('inertia ratio Izz/Ixx and thrust/weight (KDW 0.005, tau 50 ms)')
    print('Izz/Ixx  T/W   holds  spin[rad/s]  alt loss[m]')
    for ratio in [1.2, 1.5, 1.8, 2.0]:
        for tw in [1.6, 2.0, 2.45, 3.0]:
            t_max = tw * 9.81 / 4
            v = s.Vehicle(**dict(s.VEHICLES['realistic'], izz=0.01 * ratio, t_max=t_max, q_max=0.02 * t_max, w_max=40))
            ok, r, loss = holds(v)
            print('%.1f      %.2f  %-5s  %5.1f        %.1f' % (ratio, tw, ok, r, loss))


if __name__ == '__main__':
    main()
