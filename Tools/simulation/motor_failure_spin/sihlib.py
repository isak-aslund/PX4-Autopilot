"""Minimal SIH driver. Instance i: onboard link remote port 14030+i."""
import os
import re
import subprocess
import threading
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil  # noqa: E402

S = os.path.dirname(os.path.abspath(__file__))
MAIN_MANUAL, MAIN_ALTCTL, MAIN_POSCTL, MAIN_AUTO = 1, 2, 3, 4
SUB_TAKEOFF, SUB_LOITER, SUB_MISSION, SUB_RTL, SUB_LAND = 2, 3, 4, 5, 6


class Sim:
    def __init__(self, wt, inst):
        self.wt, self.inst = wt, inst
        self.bin = os.path.join(S, wt, 'build', 'px4_sitl_default', 'bin')
        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:%d' % (14030 + inst))
        self.mav.wait_heartbeat(timeout=30)
        self.sticks = [0, 0, 500, 0]
        self.sticks_enabled = True
        self.texts = []
        self.acks = {}
        self._lock = threading.Lock()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._tx, daemon=True).start()

    # --- shell ---
    def px4(self, cmd, *args, timeout=20):
        return subprocess.run([os.path.join(self.bin, 'px4-' + cmd), '--instance', str(self.inst), *map(str, args)],
                              capture_output=True, text=True, timeout=timeout).stdout

    def param(self, name, value):
        self.px4('param', 'set', name, value)

    def param_get(self, name):
        m = re.search(r'%s \[[\d,]+\] : (\S+)' % name, self.px4('param', 'show', name))
        return float(m.group(1)) if m else None

    def listener(self, topic, idx=0):
        out = self.px4('listener', topic, *(['-i', idx] if idx else []), '-n', 1)
        f = {}
        for line in out.splitlines():
            m = re.match(r'\s+(\w+): (.*)$', line)
            if m and m.group(1) not in f:
                f[m.group(1)] = m.group(2).strip()
        return f

    # --- mavlink ---
    def _rx(self):
        while True:
            if getattr(self, 'rx_paused', False):
                time.sleep(0.05)
                continue
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'STATUSTEXT':
                with self._lock:
                    self.texts.append(msg.text)
            elif t == 'COMMAND_ACK':
                with self._lock:
                    self.acks[msg.command] = msg.result

    def _tx(self):
        last_hb = 0
        while True:
            now = time.time()
            if now - last_hb > 1:
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now
            if self.sticks_enabled:
                x, y, z, r = self.sticks
                self.mav.mav.manual_control_send(self.mav.target_system, x, y, z, r, 0)
            time.sleep(0.02)

    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, timeout=5):
        with self._lock:
            self.acks.pop(cmd, None)
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component, cmd, 0, p1, p2, p3, p4, p5, p6, p7)
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if cmd in self.acks:
                    return self.acks[cmd]
            time.sleep(0.05)
        return None

    def set_mode(self, main, sub=0):
        return self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, main, sub)

    def arm(self, force=False):
        return self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196 if force else 0)

    def disarm(self, force=False):
        return self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196 if force else 0)

    def pop_texts(self):
        with self._lock:
            t, self.texts = self.texts, []
        return t

    # --- state ---
    def status(self):
        s = self.listener('vehicle_status')
        return int(s.get('nav_state', -1)), int(s.get('arming_state', 0)) == 2

    def truth(self):
        g = self.listener('vehicle_local_position_groundtruth')
        return float(g['x']), float(g['y']), float(g['z']), float(g['vx']), float(g['vy']), float(g['vz'])

    def lpos(self):
        return self.listener('vehicle_local_position')

    def wait(self, cond, timeout, period=0.5):
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return True
            time.sleep(period)
        return False

    def ensure_landed(self):
        if self.status()[1]:
            self.command(mavutil.mavlink.MAV_CMD_NAV_LAND)
            self.wait(lambda: not self.status()[1], 120)
        time.sleep(2)

    def common_setup(self):
        self.param('COM_RC_IN_MODE', 1)
        self.param('COM_ARM_MAN_EN', 1) if self.param_get('COM_ARM_MAN_EN') is not None else None

    def latest_log(self):
        d = os.path.join(S, self.wt, 'build', 'px4_sitl_default', 'inst%d' % self.inst, 'log')
        logs = []
        for root, _, files in os.walk(d):
            logs += [os.path.join(root, f) for f in files if f.endswith('.ulg')]
        return max(logs, key=os.path.getmtime) if logs else None



# --- missions ---
import math  # noqa: E402


def home(sim):
    h = sim.listener('home_position')
    return float(h['lat']), float(h['lon']), float(h['alt'])


def ned_to_ll(lat0, lon0, n, e):
    return lat0 + n / 6378137.0 * 180 / math.pi, lon0 + e / (6378137.0 * math.cos(math.radians(lat0))) * 180 / math.pi


def upload_mission(sim, wps, takeoff_alt=10.0):
    """wps: list of (north, east, alt_rel). Item 0 = takeoff at home. Returns True on accept."""
    m = sim.mav.mav
    lat0, lon0, _ = home(sim)
    items = [(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, lat0, lon0, takeoff_alt)]
    for n, e, a in wps:
        la, lo = ned_to_ll(lat0, lon0, n, e)
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, la, lo, a))
    sim.rx_paused = True
    time.sleep(0.5)
    try:
        return _upload(sim, m, items)
    finally:
        sim.rx_paused = False


def _upload(sim, m, items):
    sim.mav.mav.mission_count_send(sim.mav.target_system, sim.mav.target_component, len(items), 0)
    sent = set()
    end = time.time() + 20
    while time.time() < end:
        msg = sim.mav.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == 'MISSION_ACK':
            return msg.type == 0
        seq = msg.seq
        cmd, la, lo, a = items[seq]
        m.mission_item_int_send(sim.mav.target_system, sim.mav.target_component, seq,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1, 0, 0, 0, float('nan'),
                                int(la * 1e7), int(lo * 1e7), a, 0)
        sent.add(seq)
    return False
