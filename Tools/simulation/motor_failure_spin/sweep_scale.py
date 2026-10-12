#!/usr/bin/env python3
"""Python-model sweep: does the spin recovery scale to a large, slow quad?

The 1 kg test vehicle is scaled geometrically by a length factor s: mass s^3, inertia s^5, arm and
torque/thrust ratio s, and yaw damping s^4.5 (Froude scaling, which keeps the dynamics similar with
time scaled by sqrt(s)). The spin rate then drops as 1/sqrt(s). Real large quads have slower motors
and often less thrust margin, so those are swept separately.
"""
import numpy as np
import spin_model as s
from sweep import holds

BASE = dict(s.VEHICLES['realistic'], mass=1.0, arm=0.2, kdv=1.0)


def scaled(sc, tw, tau, rate_limit=60):
    m = BASE['mass'] * sc ** 3
    t_max = tw * m * 9.81 / 4
    kappa = BASE['q_max'] / BASE['t_max'] * sc
    return s.Vehicle(mass=m, ixx=BASE['ixx'] * sc ** 5, izz=BASE['izz'] * sc ** 5, t_max=t_max, q_max=kappa * t_max,
                     arm=BASE['arm'] * sc, kdw=BASE['kdw'] * sc ** 4.5, kdv=BASE['kdv'] * sc ** 2, tau=tau,
                     w_max=rate_limit)


if __name__ == '__main__':
    print('scale  mass[kg]  arm[m]  T/W   tau[ms]  spin[rad/s]  tau*spin[rad]  holds/alt loss[m]: detect 0 s | 0.1 s')
    for sc in [1.0, 2.0, 2.75]:
        for tw in [1.8, 2.0, 2.45]:
            for tau in [0.05, 0.1, 0.15, 0.2, 0.3]:
                v = scaled(sc, tw, tau)
                # the LQR torque weight is in (N m)^-2: scale it with the torque, which grows as s^4
                kw = dict(ctrl_kw=dict(r_diag=(400 / sc ** 8, 400 / sc ** 8)), t_end=30)
                ok0, r, loss0 = holds(v, detect=0.0, **kw)
                ok1, _, loss1 = holds(v, detect=0.1, **kw)
                print('%.2f   %6.1f    %.2f    %.2f  %4d     %5.1f        %4.1f           %-5s %5.1f | %-5s %5.1f' % (
                    sc, v.m, v.L, tw, tau * 1000, abs(r), abs(r) * tau, ok0, loss0, ok1, loss1), flush=True)
