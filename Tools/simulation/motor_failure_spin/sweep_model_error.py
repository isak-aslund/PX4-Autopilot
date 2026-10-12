#!/usr/bin/env python3
"""Python-model sweep: how wrong may the controller's vehicle model be?

The controller gets a copy of the vehicle with one parameter (or all of them at random) scaled,
while the simulated vehicle keeps the true values. Detection takes 0.1 s, so the recovery from the
tumble the nominal controller causes in that time is part of the test. Also checks a vehicle whose
yaw damping is quadratic in the spin rate while the controller assumes it is linear.

In PX4 the thrust setpoint passes through normalised; MC_SPN_TMAX only converts it to newtons
for the equilibrium and scales the torque, together with MC_SPN_ARM (arm x max thrust). Here a
wrong t_max would also scale the actual thrust, so it is left out: the ARM row stands for both.
"""
import argparse
import copy
import multiprocessing as mp

import numpy as np
import spin_model as s

CAP = 0.9 * 2.5
DETECT = 0.1
PARAMS = ['ix', 'iz', 'kdw', 'L', 'tau', 'm']
NAMES = dict(ix='Ixx', iz='Izz', kdw='KDW', L='ARM x TMAX', tau='TAU', m='mass')
FACTORS = [0.5, 0.7, 0.8, 1.25, 1.5, 2.0]


TRUE_TAU = 0.05


def true_vehicle():
    return s.Vehicle(**dict(s.VEHICLES['realistic'], tau=TRUE_TAU, w_max=40))


def believed(v, scale):
    b = copy.copy(v)
    for p, f in scale.items():
        setattr(b, p, getattr(v, p) * f)
    b.I = np.diag([b.ix, b.ix, b.iz])
    b.Iinv = np.linalg.inv(b.I)
    b.kappa = v.kappa
    return b


class QuadraticYawDamping:
    """Damping torque: linear in roll/pitch rate, quadratic in yaw rate, equal to the linear value at r0."""

    def __init__(self, kdw, r0):
        self.kdw, self.c = kdw, kdw / r0

    def __rmul__(self, w):
        return self.__mul__(w)

    def __mul__(self, w):
        w = np.asarray(w)
        return np.array([self.kdw * w[0], self.kdw * w[1], self.c * w[2] * abs(w[2])])


def _set_tau(tau):
    global TRUE_TAU
    TRUE_TAU = tau


def run(args):
    scale, plant = args
    v = true_vehicle()
    c = s.LqrSpinController(believed(v, scale), f1_ratio=0.5)
    if plant == 'quadratic':
        v.kdw = QuadraticYawDamping(v.kdw, 23.5)
    log = s.simulate(v, c, t_end=25, delay=0.005, detect=DETECT, thrust_cap=CAP * v.t_max)
    t, pos, u = log[:, 0], log[:, 1:4], log[:, 10:14]
    if t[-1] < 24.9:
        return 'crash', pos[:, 2].max()
    late = t > 21
    ok = np.abs(pos[late]).max() < 1.0 and np.ptp(u[late, 1:], axis=0).max() < 0.1
    return ('ok' if ok else 'osc'), pos[:, 2].max()


def main():
    global TRUE_TAU
    ap = argparse.ArgumentParser()
    ap.add_argument('--tau', type=float, default=0.05, help='true motor time constant [s]')
    TRUE_TAU = ap.parse_args().tau
    rng = np.random.default_rng(1)
    jobs = [({}, 'linear'), ({}, 'quadratic')]
    jobs += [({p: f}, 'linear') for p in PARAMS for f in FACTORS]
    joint = {}
    for spread in (0.25, 0.5):
        draws = [{p: float(np.exp(rng.uniform(np.log(1 - spread), np.log(1 + spread)))) for p in PARAMS}
                 for _ in range(30)]
        joint[spread] = draws
        jobs += [(d, 'linear') for d in draws]
    with mp.Pool(initializer=_set_tau, initargs=(TRUE_TAU,)) as pool:
        res = pool.map(run, jobs)
    it = iter(res)
    base, quad = next(it), next(it)
    print('detection %.1f s, true motor tau %d ms; altitude loss of 20 m is a crash' % (DETECT, TRUE_TAU * 1000))
    print('exact model:                    %s, lost %.1f m' % base)
    print('yaw damping quadratic in plant: %s, lost %.1f m' % quad)
    print()
    print('one parameter wrong, model value / true value:')
    print('param       ' + ''.join('%-12s' % ('x%.2f' % f) for f in FACTORS))
    for p in PARAMS:
        print('%-11s ' % NAMES[p] + ''.join('%-5s %5.1f ' % next(it) for _ in FACTORS))
    for spread, draws in joint.items():
        r = [next(it) for _ in draws]
        oks = [x for x in r if x[0] == 'ok']
        print()
        print('all parameters wrong at random, each within +-%d %%: %d/%d ok, %d osc, %d crash; loss median %.1f m, worst ok %.1f m'
              % (spread * 100, len(oks), len(r), sum(x[0] == 'osc' for x in r), sum(x[0] == 'crash' for x in r),
                 np.median([x[1] for x in r]), max([x[1] for x in oks]) if oks else float('nan')))
        bad = [(d, x) for d, x in zip(draws, r) if x[0] != 'ok']
        for d, x in bad:
            print('   %s %4.1f m: ' % x + ' '.join('%s %.2f' % (NAMES[p], d[p]) for p in PARAMS))


if __name__ == '__main__':
    main()
