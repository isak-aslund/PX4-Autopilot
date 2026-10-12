#!/usr/bin/env python3
"""Plot a motor-failure SIH log: ana.py <log.ulg> <out.png> [t_fail_offset_s]"""
import sys

import numpy as np
from pyulog import ULog

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def topic(ulog, name, idx=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == idx:
            return d.data
    return None


def main():
    ulog = ULog(sys.argv[1])
    t0 = ulog.start_timestamp

    for m in ulog.logged_messages:
        if any(k in m.message for k in ('otor', 'spin', 'ailure', 'Removing')):
            print('%.2f %s' % ((m.timestamp - t0) / 1e6, m.message))

    cas = topic(ulog, 'control_allocator_status')
    t_fail = None
    if cas is not None:
        i = np.nonzero(cas['handled_motor_failure_mask'])[0]
        if len(i):
            t_fail = (cas['timestamp'][i[0]] - t0) / 1e6
            print('allocator handles failure at %.2f s' % t_fail)

    gt = topic(ulog, 'vehicle_local_position_groundtruth')
    w = topic(ulog, 'vehicle_angular_velocity')
    am = topic(ulog, 'actuator_motors')
    tq = topic(ulog, 'vehicle_torque_setpoint')
    th = topic(ulog, 'vehicle_thrust_setpoint')
    att = topic(ulog, 'vehicle_attitude_groundtruth') or topic(ulog, 'vehicle_attitude')
    sp = topic(ulog, 'vehicle_attitude_setpoint')

    tt = lambda d: (d['timestamp'] - t0) / 1e6  # noqa: E731
    tmin = (t_fail or tt(gt)[0]) - 3
    tmax = tmin + float(sys.argv[3]) if len(sys.argv) > 3 else tt(gt)[-1]

    fig, ax = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    ax[0].plot(tt(gt), gt['x'], tt(gt), gt['y'], tt(gt), -gt['z'])
    ax[0].legend(['x', 'y', 'alt'])
    ax[0].set_ylabel('truth [m]')
    ax[1].plot(tt(w), w['xyz[0]'], tt(w), w['xyz[1]'], tt(w), w['xyz[2]'])
    ax[1].legend(['p', 'q', 'r'])
    ax[1].set_ylabel('rate [rad/s]')
    for i in range(4):
        ax[2].plot(tt(am), am['control[%d]' % i], label='m%d' % (i + 1))
    ax[2].legend()
    ax[2].set_ylabel('motor cmd')
    ax[3].plot(tt(tq), tq['xyz[0]'], tt(tq), tq['xyz[1]'], tt(tq), tq['xyz[2]'], tt(th), th['xyz[2]'])
    ax[3].legend(['tx', 'ty', 'tz', 'thrust z'])
    ax[3].set_ylabel('setpoints')

    # tilt of body z from vertical, and of the thrust setpoint
    def tilt(q0, q1, q2, q3):
        return np.degrees(np.arccos(np.clip(1 - 2 * (q1 * q1 + q2 * q2), -1, 1)))
    ax[4].plot(tt(att), tilt(att['q[0]'], att['q[1]'], att['q[2]'], att['q[3]']), label='body z tilt')
    ax[4].plot(tt(sp), tilt(sp['q_d[0]'], sp['q_d[1]'], sp['q_d[2]'], sp['q_d[3]']), label='setpoint tilt')
    ax[4].legend()
    ax[4].set_ylabel('deg')
    ax[5].plot(tt(gt), gt['vx'], tt(gt), gt['vy'], tt(gt), gt['vz'])
    ax[5].legend(['vx', 'vy', 'vz'])
    ax[5].set_ylabel('truth vel')
    for a in ax:
        a.grid(True)
        if t_fail:
            a.axvline(t_fail, color='k', ls=':')
    ax[-1].set_xlim(tmin, tmax)
    ax[-1].set_xlabel('t [s]')
    fig.tight_layout()
    fig.savefig(sys.argv[2], dpi=80)


if __name__ == '__main__':
    main()
