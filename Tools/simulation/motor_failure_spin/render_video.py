#!/usr/bin/env python3
"""
Render a motor-failure SIH log as a video: chase-camera 3D view of the quad plus time strips.

usage: render_video.py <log.ulg> <out.mp4> [--fps 30] [--slow 0.25] [--fast 2]
Playback: real time before the failure, slow motion around it, sped up during the hold and landing.
Everything drawn comes from the log: ground-truth attitude and position, ESC RPM (what the motors
really did), and the logged events.
"""
import argparse
import os
import shutil
import subprocess
import tempfile
from multiprocessing import Pool

import numpy as np
from pyulog import ULog

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

ARM = 0.2          # motor offset along body x and y [m] (SIH_L_ROLL)
ROTOR_R = 0.11     # drawn rotor radius [m]
MOTORS_B = np.array([[ARM, ARM], [-ARM, -ARM], [ARM, -ARM], [-ARM, ARM]])  # PX4 quad X, FRD
MAX_RPM = 10000.0  # SIH throttle_to_rpm scale

D = {}  # log data, filled in load() and inherited by the workers


def topic(ulog, name):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    raise KeyError(name)


def load(path):
    u = ULog(path)
    t0 = u.start_timestamp
    att = topic(u, 'vehicle_attitude_groundtruth')
    pos = topic(u, 'vehicle_local_position_groundtruth')
    esc = topic(u, 'esc_status')
    am = topic(u, 'actuator_motors')
    w = topic(u, 'vehicle_angular_velocity_groundtruth')
    ev = {}
    for m in u.logged_messages:
        t = (m.timestamp - t0) / 1e6
        for key, pat in (('fail', 'SIH: motor'), ('detect', 'Removing motor'), ('land', 'Landing at current'),
                         ('landed', 'Landing detected'), ('disarm', 'Disarmed')):
            if pat in m.message and key not in ev:
                ev[key] = t
    D.update(
        ev=ev,
        ta=(att['timestamp'] - t0) / 1e6, q=np.stack([att['q[%d]' % i] for i in range(4)], 1),
        tp=(pos['timestamp'] - t0) / 1e6, p=np.stack([pos['x'], pos['y'], pos['z']], 1),
        te=(esc['timestamp'] - t0) / 1e6, rpm=np.stack([esc['esc[%d].esc_rpm' % i] for i in range(4)], 1),
        tw=(w['timestamp'] - t0) / 1e6, w=np.stack([w['xyz[%d]' % i] for i in range(3)], 1),
        tc=(am['timestamp'] - t0) / 1e6, cmd=np.nan_to_num(np.stack([am['control[%d]' % i] for i in range(4)], 1)),
    )
    # ground impact after the failure
    alt = -D['p'][:, 2]
    hit = np.nonzero((D['tp'] > ev.get('fail', 1e9)) & (alt < 0.3))[0]
    if len(hit) and 'detect' not in ev:
        ev['impact'] = D['tp'][hit[0]]
    q = D['q']
    D['tilt'] = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    D['thr'] = np.clip(D['rpm'] / MAX_RPM, 0, 1)
    # what each motor really does: its command, except the failed one, which produces nothing
    D['real'] = D['cmd'].copy()
    D['real'][D['tc'] >= ev.get('fail', 1e9), 0] = 0.0


def quat_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def at(t_arr, v, t):
    i = np.clip(np.searchsorted(t_arr, t), 0, len(t_arr) - 1)
    return v[i]


def interp(t_arr, v, t):
    return np.stack([np.interp(t, t_arr, v[:, i]) for i in range(v.shape[1])], -1)


def ned_to_view(p):
    """NED -> right-handed east, north, up"""
    p = np.asarray(p)
    return np.stack([p[..., 1], p[..., 0], -p[..., 2]], -1)


class Camera:
    def __init__(self, target, dist=1.9, elev=22.0, azim=-60.0, f=1.9):
        el, az = np.radians(elev), np.radians(azim)
        self.eye = target + dist * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        fwd = target - self.eye
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, [0, 0, 1])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        self.M = np.stack([right, up, fwd])
        self.f = f

    def project(self, pts):
        c = (np.asarray(pts) - self.eye) @ self.M.T
        z = np.maximum(c[..., 2], 1e-3)
        return self.f * c[..., 0] / z, self.f * c[..., 1] / z, c[..., 2]


def phase(t):
    ev = D['ev']
    if t < ev.get('fail', 1e9):
        return 'Hovering on 4 motors (Position mode)', '#2a7d2a'
    if 'detect' not in ev:
        if t < ev.get('impact', 1e9):
            return 'Motor 1 DEAD, stock PX4: keeps fighting with 3 motors', '#b02020'
        return 'Crashed', '#222222'
    if t < ev.get('detect', 1e9):
        return 'Motor 1 DEAD, not detected yet: normal controller still drives it', '#b02020'
    if t < ev['detect'] + 5.0:
        return 'Failure detected: yaw given up, spin axis steered upright', '#c06000'
    if t < ev.get('land', 1e9):
        return 'Flying on 3 motors while spinning (position control)', '#1f5fa8'
    if t < ev.get('landed', 1e9):
        return 'Landing while spinning', '#6a3fa0'
    return 'Landed, disarmed', '#555555'


STEP_TEXT = {
    '#2a7d2a': '',
    '#b02020': 'Motor 1 produces nothing. The rate controller saturates\nthe other motors and rolls the vehicle over.',
    '#c06000': 'CA_FAILURE_MODE 3: motor 1 and yaw removed from the allocation.\n'
               'MC_SPN_EN: LQR points the spin axis along the thrust setpoint,\n'
               'thrust capped so the 3 motors keep room for control.',
    '#1f5fa8': 'Steady state: constant motor commands (opposite motor ~1/3),\n'
               'gyroscopic moment of the spin balances the uneven thrust.',
    '#6a3fa0': 'Normal Land mode on top of spin control.',
    '#555555': 'Land detector ignores roll/pitch rate after a motor failure.',
    '#222222': 'Ground impact. Stock PX4 never stops trying to hold yaw.',
}


def draw_frame(args):
    k, t, speed, out_dir, t_lo, t_hi = args
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.0, 0.0, 0.6, 1.0])
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-0.95, 0.95)
    ax.set_aspect('equal')
    ax.axis('off')
    fig.patch.set_facecolor('#eef2f6')

    p_ned = interp(D['tp'], D['p'], t)
    q = at(D['ta'], D['q'], t)
    R = quat_R(q)
    thr = at(D['tc'], D['real'], t)
    cmd = at(D['tc'], D['cmd'], t)
    pos = ned_to_view(p_ned)

    # camera target lags the vehicle a little (smooth)
    tgt = ned_to_view(interp(D['tp'], D['p'], np.linspace(t - 0.08, t, 5)).mean(0))
    cam = Camera(tgt)

    polys = []  # (depth, xy, kwargs)

    # ground grid around the vehicle, with its shadow
    gx0, gy0 = np.round(pos[0]), np.round(pos[1])
    for d in np.arange(-6, 7):
        for a, b in ((np.array([gx0 + d, gy0 - 6, 0]), np.array([gx0 + d, gy0 + 6, 0])),
                     (np.array([gx0 - 6, gy0 + d, 0]), np.array([gx0 + 6, gy0 + d, 0]))):
            seg = np.linspace(a, b, 30)
            u, v, z = cam.project(seg)
            m = z > 0.2
            ax.plot(u[m], v[m], color='#b8c4cf', lw=0.6, zorder=1)

    # vertical reference and altitude pole down to the ground
    u, v, _ = cam.project(np.array([pos, [pos[0], pos[1], 0.0]]))
    ax.plot(u, v, color='#9aa7b3', lw=0.8, ls=':', zorder=2)

    # trail
    tt = np.linspace(max(t - 3.0, D['tp'][0]), t, 60)
    trail = ned_to_view(interp(D['tp'], D['p'], tt))
    u, v, _ = cam.project(trail)
    ax.plot(u, v, color='#7d8fa3', lw=1.2, alpha=0.7, zorder=2)

    def body(pb):
        return ned_to_view(p_ned + R @ pb)

    # arms
    for i, (mx, my) in enumerate(MOTORS_B):
        a, b = body(np.zeros(3)), body(np.array([mx, my, 0.0]))
        front = mx > 0
        u, v, z = cam.project(np.array([a, b]))
        polys.append((z.mean(), ('line', u, v), dict(color='#e07b00' if front else '#3b4650', lw=4)))

    # rotors: discs whose opacity follows the real motor speed
    ang = np.linspace(0, 2 * np.pi, 28)
    dead = t >= D['ev'].get('fail', 1e9)
    for i, (mx, my) in enumerate(MOTORS_B):
        ring = np.stack([mx + ROTOR_R * np.cos(ang), my + ROTOR_R * np.sin(ang), np.full_like(ang, -0.03)], 1)
        pts = np.array([body(r) for r in ring])
        u, v, z = cam.project(pts)
        if i == 0 and dead:
            kw = dict(facecolor='#ff3030', alpha=0.25, edgecolor='#c00000', lw=2)
        else:
            kw = dict(facecolor='#2f80d0', alpha=0.15 + 0.6 * thr[i], edgecolor='#1a4f86', lw=1)
        polys.append((z.mean() + 0.001, ('poly', u, v), kw))
        c = cam.project(body(np.array([mx, my, -0.03])))
        label = ('M%d DEAD (cmd %d%%)' % (i + 1, round(100 * cmd[i]))) if (i == 0 and dead) else \
            'M%d %d%%' % (i + 1, round(100 * thr[i]))
        polys.append((c[2] - 0.05, ('text', c[0], c[1], label),
                      dict(color='#b00000' if (i == 0 and dead) else '#13324f', fontsize=9, ha='center',
                           va='center', weight='bold')))

    # thrust axis (body up) and world vertical
    a = body(np.zeros(3))
    for vec, col, lab in ((R @ np.array([0, 0, -0.55]), '#00a0a0', 'thrust axis'), (np.array([0, 0, -0.55]), '#888888', 'vertical')):
        b = ned_to_view(p_ned + vec)
        u, v, z = cam.project(np.array([a, b]))
        polys.append((z.mean() - 0.3, ('arrow', u, v, lab), dict(color=col)))

    # centre body
    u, v, z = cam.project(a)
    polys.append((z, ('dot', u, v), dict(color='#222', ms=10)))

    for _, item, kw in sorted(polys, key=lambda x: -x[0]):
        kind = item[0]
        if kind == 'line':
            ax.plot(item[1], item[2], solid_capstyle='round', zorder=5, **kw)
        elif kind == 'poly':
            ax.add_patch(Polygon(np.stack([item[1], item[2]], 1), closed=True, zorder=5, **kw))
        elif kind == 'text':
            ax.text(item[1], item[2], item[3], zorder=9, **kw)
        elif kind == 'arrow':
            ax.annotate('', xy=(item[1][1], item[2][1]), xytext=(item[1][0], item[2][0]),
                        arrowprops=dict(arrowstyle='-|>', color=kw['color'], lw=2), zorder=8)
            ax.text(item[1][1], item[2][1], ' ' + item[3], color=kw['color'], fontsize=8, zorder=8)
        elif kind == 'dot':
            ax.plot([item[1]], [item[2]], 'o', zorder=6, **kw)

    # overlay text
    title, col = phase(t)
    ax.text(-0.97, 0.9, title, fontsize=13, color='white', weight='bold', va='top',
            bbox=dict(boxstyle='round,pad=0.4', fc=col, ec='none'))
    ax.text(-0.97, 0.78, STEP_TEXT[col], fontsize=9.5, color='#222', va='top', family='monospace')
    w = at(D['tw'], D['w'], t)
    tilt = at(D['ta'], D['tilt'], t)
    ax.text(-0.97, -0.92, 't = %5.2f s   altitude %5.1f m   yaw rate %5.1f rad/s (%.1f rev/s)   tilt %4.0f°'
            % (t, pos[2], w[2], abs(w[2]) / (2 * np.pi), tilt), fontsize=10, family='monospace', color='#13324f')
    ax.text(0.97, 0.9, '%sx' % ('%g' % speed), fontsize=16, ha='right', va='top',
            color='#b02020' if speed < 1 else ('#1f5fa8' if speed > 1 else '#444'), weight='bold')
    ax.text(0.97, 0.82, 'slow motion' if speed < 1 else ('fast forward' if speed > 1 else 'real time'),
            fontsize=9, ha='right', va='top', color='#555')
    ax.text(0.97, -0.97, 'PX4 SIH, motor 1 fails at 19 m, chase camera, ground truth', fontsize=8,
            ha='right', va='bottom', color='#777')

    # top-down map: whole flight faint, flown part solid
    mp = fig.add_axes([0.43, 0.16, 0.16, 0.26])
    m_all = (D['tp'] >= t_lo) & (D['tp'] <= t_hi)
    m_now = m_all & (D['tp'] <= t)
    e_all, n_all = D['p'][m_all, 1], D['p'][m_all, 0]
    mp.plot(e_all, n_all, color='#c8d0d8', lw=1)
    mp.plot(D['p'][m_now, 1], D['p'][m_now, 0], color='#1f5fa8', lw=1.5)
    mp.plot([p_ned[1]], [p_ned[0]], 'o', color='#b02020', ms=5)
    pad = 3
    lo_e, hi_e, lo_n, hi_n = e_all.min() - pad, e_all.max() + pad, n_all.min() - pad, n_all.max() + pad
    span = max(hi_e - lo_e, hi_n - lo_n, 10)
    ce, cn = (lo_e + hi_e) / 2, (lo_n + hi_n) / 2
    mp.set_xlim(ce - span / 2, ce + span / 2)
    mp.set_ylim(cn - span / 2, cn + span / 2)
    mp.set_aspect('equal')
    mp.tick_params(labelsize=6)
    mp.set_title('top view (N up) [m]', fontsize=7)
    mp.grid(alpha=0.3)

    # strips
    strips = [
        ('altitude [m]', D['tp'], -D['p'][:, 2], None),
        ('yaw rate [rad/s]', D['tw'], D['w'][:, 2], None),
        ('tilt [deg]', D['ta'], D['tilt'], None),
        ('motor command [%]', D['tc'], 100 * D['cmd'], ['M1 (dead from fail)', 'M2 (opposite)', 'M3', 'M4']),
    ]
    colors = ['#d62728', '#ff9f1c', '#2f80d0', '#6a3fa0']
    for j, (lab, tx, y, legend) in enumerate(strips):
        sa = fig.add_axes([0.66, 0.77 - j * 0.235, 0.32, 0.19])
        m = (tx >= t_lo) & (tx <= t_hi)
        if y.ndim == 1:
            sa.plot(tx[m], y[m], color='#13324f', lw=1)
        else:
            for i in range(y.shape[1]):
                sa.plot(tx[m], y[m, i], color=colors[i], lw=0.8, label=legend[i])
            sa.legend(fontsize=7, loc='upper right', ncol=2, framealpha=0.8)
        sa.axvline(t, color='#b02020', lw=1.2)
        for key in ('fail', 'detect', 'land', 'impact'):
            if key in D['ev']:
                sa.axvline(D['ev'][key], color='#999', lw=0.7, ls='--')
        sa.set_xlim(t_lo, t_hi)
        sa.set_ylabel(lab, fontsize=8)
        sa.tick_params(labelsize=7)
        sa.grid(alpha=0.3)
        if j < len(strips) - 1:
            sa.set_xticklabels([])
        else:
            sa.set_xlabel('log time [s]', fontsize=8)

    fig.savefig(os.path.join(out_dir, 'f%05d.png' % k), facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('out')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--slow', type=float, default=0.25)
    ap.add_argument('--fast', type=float, default=2.0)
    ap.add_argument('--jobs', type=int, default=16)
    a = ap.parse_args()

    load(a.log)
    ev = D['ev']
    t_lo = ev['fail'] - 4.0
    t_hi = ev['impact'] + 1.5 if 'impact' in ev else ev.get('disarm', D['tp'][-1]) + 3.0
    slow_lo, slow_hi = ev['fail'] - 0.5, ev.get('detect', ev['fail']) + 4.0

    # playback schedule: (log time, speed) per frame
    frames, t = [], t_lo
    while t < t_hi:
        speed = a.slow if slow_lo <= t < slow_hi else (1.0 if t < slow_lo else a.fast)
        frames.append((t, speed))
        t += speed / a.fps

    tmp = tempfile.mkdtemp(prefix='spinvid_')
    jobs = [(k, t, s, tmp, t_lo, t_hi) for k, (t, s) in enumerate(frames)]
    print('%d frames, %.0f s of video' % (len(frames), len(frames) / a.fps))
    with Pool(a.jobs) as pool:
        for i, _ in enumerate(pool.imap_unordered(draw_frame, jobs, chunksize=4)):
            if i % 200 == 0:
                print('  %d/%d' % (i, len(jobs)), flush=True)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', os.path.join(tmp, 'f%05d.png'),
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', '-movflags', '+faststart', a.out], check=True)
    shutil.rmtree(tmp)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
