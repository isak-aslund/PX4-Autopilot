#!/usr/bin/env python3
"""
SIH test: quadrotor hovers in Position mode at 20 m, motor 1 dies (SIH_MOT_FAIL), then Land.

usage: spin_sih.py <mode> [--vehicle realistic|sihdefault] [--inst 7] [--land-after 25]
modes: stock  - CA_FAILURE_MODE 0 (allocator keeps commanding the dead motor)
       remove - CA_FAILURE_MODE 1 (motor removed from the allocation, yaw still controlled)
       spin   - CA_FAILURE_MODE 3 + MC_SPN_EN 1 (give up yaw, spin, reduced attitude LQR)
"""
import argparse
import json
import shutil
import math
import os
import subprocess
import time

from sihlib import Sim, MAIN_POSCTL, MAIN_AUTO, SUB_TAKEOFF, S, home, ned_to_ll
from pymavlink import mavutil

# repository root: the SIH build in build/px4_sitl_default of this checkout is used
WT = os.path.abspath(os.path.join(S, '..', '..', '..'))

VEHICLES = {
    # plausible 1 kg X quad: flat mass distribution (Izz ~ 2 Ixx), little rotational drag, T/W 2.45
    'realistic': dict(SIH_IXX=0.01, SIH_IYY=0.01, SIH_IZZ=0.018, SIH_KDW=0.005, SIH_T_MAX=6.0, SIH_Q_MAX=0.12,
                      MC_SPN_IXX=0.01, MC_SPN_IZZ=0.018, MC_SPN_KDW=0.005, MC_SPN_TMAX=6.0),
    # SIH defaults: Izz only 1.2 Ixx, strong rotational damping, T/W 2.04
    'sihdefault': dict(MC_SPN_IXX=0.025, MC_SPN_IZZ=0.03, MC_SPN_KDW=0.025, MC_SPN_TMAX=5.0),
}

MODES = {
    'stock': dict(CA_FAILURE_MODE=0, MC_SPN_EN=0),
    'remove': dict(CA_FAILURE_MODE=1, MC_SPN_EN=0),
    'spin': dict(CA_FAILURE_MODE=3, MC_SPN_EN=1),
}

COMMON = dict(FD_ACT_EN=1, MOTFAIL_TIME=0.3, COM_ACT_FAIL_ACT=0, SIH_W_MAX=30.0, MC_SPN_ARM=0.2, MC_SPN_TAU=0.05,
              MIS_TAKEOFF_ALT=20.0, SDLOG_PROFILE=17, COM_RC_IN_MODE=1, NAV_RCL_ACT=0, NAV_DLL_ACT=0,
              COM_DISARM_LAND=2.0)


def bin_dir():
    return os.path.join(S, WT, 'build', 'px4_sitl_default', 'bin')


def kill_instance(inst):
    for p in subprocess.run(['pgrep', '-x', 'px4'], capture_output=True, text=True).stdout.split():
        try:
            exe = os.readlink('/proc/%s/exe' % p)
            cmd = open('/proc/%s/cmdline' % p).read().split('\0')
        except OSError:
            continue
        if exe.startswith(WT + os.sep) and '-i' in cmd and cmd[cmd.index('-i') + 1] == str(inst):
            os.kill(int(p), 15)
    time.sleep(3)


def start(inst, keep):
    env = dict(os.environ)
    if keep:
        env['KEEP'] = '1'
    subprocess.Popen(['setsid', os.path.join(S, 'sih.sh'), WT, str(inst)], env=env, stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
    time.sleep(5)


def wait_ready(sim, timeout=120):
    return sim.wait(lambda: sim.lpos().get('xy_valid') == 'True', timeout, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=MODES)
    ap.add_argument('--vehicle', default='realistic', choices=VEHICLES)
    ap.add_argument('--inst', type=int, default=7)
    ap.add_argument('--land-after', type=float, default=25.0)
    ap.add_argument('--extra', default='', help='extra params NAME=VALUE,...')
    ap.add_argument('--motor', type=int, default=1, help='motor that fails (1-based)')
    ap.add_argument('--tag', default='')
    ap.add_argument('--moves', action='store_true',
                    help='after recovery: go-to 20 m N, 20 m E, home, then full forward stick in Position mode')
    a = ap.parse_args()

    params = dict(COMMON, **VEHICLES[a.vehicle], **MODES[a.mode])
    for kv in filter(None, a.extra.split(',')):
        k, v = kv.split('=')
        params[k] = float(v)

    # fresh instance, set parameters, restart so that the logger and estimator pick them up
    kill_instance(a.inst)
    start(a.inst, keep=False)
    sim = Sim(WT, a.inst)
    assert wait_ready(sim), 'no position'
    for k, v in params.items():
        sim.param(k, v)
    sim.px4('param', 'save')
    time.sleep(2)
    sim.mav.close()
    kill_instance(a.inst)
    start(a.inst, keep=True)
    sim = Sim(WT, a.inst)
    assert wait_ready(sim), 'no position after restart'
    for k, v in params.items():
        got = sim.param_get(k)
        assert got is not None and abs(got - v) < 1e-4, 'param %s is %s, wanted %s' % (k, got, v)
    time.sleep(5)

    # take off and hold in Position mode
    sim.set_mode(MAIN_AUTO, SUB_TAKEOFF)
    sim.arm()
    assert sim.wait(lambda: -float(sim.truth()[2]) > 19.0, 60), 'takeoff failed'
    sim.set_mode(MAIN_POSCTL)
    time.sleep(8)
    sim.pop_texts()

    x0, y0, z0, *_ = sim.truth()
    print('hover at %.1f m, motor %d fails' % (-z0, a.motor))
    sim.param('SIH_MOT_FAIL', a.motor)
    t0 = time.time()
    samples = []
    landing = False
    lat0, lon0, _ = home(sim)
    # (time after failure, action)
    moves = [(10, ('goto', 20, 0)), (22, ('goto', 20, 20)), (34, ('goto', 0, 0)),
             (46, ('stick', 1000)), (50, ('stick', 0))] if a.moves else []
    impact = None

    while time.time() - t0 < a.land_after + 60:
        t = time.time() - t0
        try:
            x, y, z, vx, vy, vz = sim.truth()
            w = sim.listener('vehicle_angular_velocity')
            r = float(w['xyz'].strip('[]').split(',')[2]) if 'xyz' in w else float('nan')
        except (KeyError, ValueError, subprocess.TimeoutExpired):
            continue
        samples.append((t, x - x0, y - y0, z, vx, vy, vz, r))

        if -z < 0.3 and impact is None:
            impact = (t, math.sqrt(vx * vx + vy * vy + vz * vz), vz)
            print('t=%.1f on the ground, speed %.1f m/s (vz %.1f)' % (t, impact[1], vz))

        while moves and t > moves[0][0] and impact is None:
            _, act = moves.pop(0)
            if act[0] == 'goto':
                la, lo = ned_to_ll(lat0, lon0, act[1], act[2])
                print('t=%.1f go to N %d E %d' % (t, act[1], act[2]))
                sim.mav.mav.command_int_send(sim.mav.target_system, sim.mav.target_component, 0,
                                             mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1, 1, 0, float('nan'),
                                             int(la * 1e7), int(lo * 1e7), float('nan'))
            else:
                if act[1]:
                    sim.set_mode(MAIN_POSCTL)
                print('t=%.1f Position mode, pitch stick %d' % (t, act[1]))
                sim.sticks = [act[1], 0, 500, 0]

        if not landing and t > a.land_after and impact is None:
            print('t=%.1f commanding Land' % t)
            sim.command(mavutil.mavlink.MAV_CMD_NAV_LAND)
            landing = True

        if impact is not None and not sim.status()[1]:
            print('t=%.1f disarmed' % t)
            break

        if impact is not None and t - impact[0] > 10:
            break

    texts = sim.pop_texts()
    flying = [s for s in samples if s[0] < (impact[0] if impact else 1e9)]
    hold = [s for s in flying if 5 < s[0] < a.land_after]
    summary = dict(
        mode=a.mode, vehicle=a.vehicle, tag=a.tag, motor=a.motor, extra=a.extra,
        impact_t=impact[0] if impact else None, impact_speed=impact[1] if impact else None,
        alt_loss_first_5s=max(s[3] - z0 for s in flying if s[0] < 5) if flying else None,
        max_horiz_dev=max(math.hypot(s[1], s[2]) for s in flying) if flying else None,
        hold_horiz_dev=max(math.hypot(s[1], s[2]) for s in hold) if hold else None,
        hold_alt_dev=max(abs(s[3] - z0) for s in hold) if hold else None,
        yaw_rate_hold=sum(s[7] for s in hold) / len(hold) if hold else None,
        log=sim.latest_log(),
        texts=[t for t in texts if 'otor' in t or 'spin' in t or 'ailure' in t][:8],
    )
    os.makedirs(os.path.join(S, 'logs'), exist_ok=True)
    if summary['log']:
        sim.disarm(force=True)
        time.sleep(2)
        dst = os.path.join(S, 'logs', 'result_%s_%s%s.ulg' % (a.mode, a.vehicle, ('_' + a.tag) if a.tag else ''))
        shutil.copy(summary['log'], dst)
        summary['log'] = dst
    print(json.dumps(summary, indent=1))
    out = os.path.join(S, 'result_%s_%s%s.json' % (a.mode, a.vehicle, ('_' + a.tag) if a.tag else ''))
    json.dump(dict(summary=summary, samples=samples), open(out, 'w'))
    sim.disarm(force=True)
    kill_instance(a.inst)


if __name__ == '__main__':
    main()
