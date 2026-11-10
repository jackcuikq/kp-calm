#!/usr/bin/env python3
"""Closed-loop simulator for team_navigation — no ROS, no robot.

Runs the real Navigation class from navigation.py (process_image ->
update_estimate -> choose_action -> compute_motion) against a synthetic
camera looking at a synthetic road, moves a unicycle car with the returned
(v, yaw), and renders the next frame. Lets you tune perception thresholds
and PD gains on a laptop.

    python tools/sim.py                       # live window, Esc/q to quit
    python tools/sim.py --headless --seconds 40 --video run.mp4 --plot run.png
    python tools/sim.py --track figure8 --latency 3 --speed 0.15
    python tools/sim.py --set kp_rad_s_per_px=0.03 --set kd_rad_per_px=0.004

Perception: by default the car uses Navigation.process_image; while that
still returns None, the simulator substitutes its own reference detector
(HSV threshold on the yellow road) so control can be tuned first.
--perception ref forces the reference, --perception node forces yours.

Not a physics engine: no slip, no motor dynamics beyond a first-order lag,
camera is an ideal pinhole. Use it for signs, gains, latency tolerance and
the offset-over-time plot the handout asks for, then validate on the car.
"""
import argparse
import csv
import math
import sys
import time
import types
from collections import deque
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NAV_DIR = ROOT / 'robot' / 'src' / 'team_navigation' / 'team_navigation'
YAML = ROOT / 'robot' / 'src' / 'team_navigation' / 'config' / 'navigation.yaml'

# ----------------------------------------------------------------------------- fake ROS
class SimTime:
    def __init__(self, ns): self.nanoseconds = int(ns)
    def __sub__(self, o): return SimTime(self.nanoseconds - o.nanoseconds)
    def to_msg(self): return self
    @staticmethod
    def from_msg(m): return m

class SimClock:
    t = 0.0
    def now(self): return SimTime(self.t * 1e9)

CLOCK = SimClock()

class _Param:
    def __init__(self, v): self.value = v

class _Logger:
    def _p(self, lvl, m): print(f'[{lvl}] {m}', file=sys.stderr)
    def info(self, m): self._p('INFO', m)
    def warn(self, m): self._p('WARN', m)
    def error(self, m): self._p('ERROR', m)
    def debug(self, m): pass

class _Pub:
    def __init__(self): self.last = None
    def publish(self, m): self.last = m
    def get_subscription_count(self): return 0

class FakeNode:
    params = {}
    def __init__(self, name): self._name = name
    def declare_parameter(self, name, default=None):
        return _Param(self.params.get(name, default))
    def create_subscription(self, *a, **k): return None
    def create_publisher(self, *a, **k): return _Pub()
    def create_timer(self, *a, **k): return None
    def get_clock(self): return CLOCK
    def get_logger(self): return _Logger()
    def destroy_node(self): pass

def install_fake_ros():
    def mod(name, **attrs):
        m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
    mod('rclpy', init=lambda *a, **k: None, spin=lambda n: None, ok=lambda: False, shutdown=lambda: None)
    mod('rclpy.node', Node=FakeNode)
    mod('rclpy.qos', qos_profile_sensor_data=None, QoSProfile=lambda **k: None,
        ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1))
    mod('rclpy.executors', ExternalShutdownException=Exception)
    mod('rclpy.time', Time=SimTime)
    mod('sensor_msgs'); mod('sensor_msgs.msg', Image=object, Imu=object, CompressedImage=types.SimpleNamespace)
    mod('cv_bridge', CvBridge=lambda: None)
    mod('course_robot_interfaces'); mod('course_robot_interfaces.msg', MotionRequest=types.SimpleNamespace)

def load_yaml_params(path):
    """Tiny parser for the flat `key: value` lines of navigation.yaml (no PyYAML needed)."""
    params = {}
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if ':' not in line or line.endswith(':'):
            continue
        k, v = (s.strip() for s in line.split(':', 1))
        try:
            params[k] = float('nan') if v in ('.nan', 'nan') else (int(v) if v.lstrip('-').isdigit() else float(v))
        except ValueError:
            params[k] = v.strip('"\'')
    return params

# ----------------------------------------------------------------------------- world
PPM = 100  # map pixels per metre
ROAD_BGR, FLOOR_BGR, WALL_BGR = (40, 200, 230), (120, 120, 120), (200, 200, 200)

def make_track(kind, width_m):
    """Return (map image BGR, centreline Nx2 metres, map size metres)."""
    if kind == 'oval':
        a, b, n = 2.0, 1.2, 400
        t = np.linspace(0, 2 * np.pi, n, endpoint=False)
        c = np.stack([a * np.cos(t), b * np.sin(t)], 1)
    elif kind == 'figure8':
        n = 600
        t = np.linspace(0, 2 * np.pi, n, endpoint=False)
        c = np.stack([2.2 * np.sin(t), 1.1 * np.sin(2 * t)], 1)
    elif kind == 'wiggle':
        n = 500
        t = np.linspace(0, 2 * np.pi, n, endpoint=False)
        r = 1.6 + 0.35 * np.sin(3 * t) + 0.15 * np.cos(5 * t)
        c = np.stack([r * np.cos(t), r * np.sin(t)], 1)
    else:
        raise SystemExit(f'unknown track {kind}')
    c -= c.min(0) - 0.8  # margin
    size = c.max(0) + 0.8
    img = np.full((int(size[1] * PPM), int(size[0] * PPM), 3), FLOOR_BGR, np.uint8)
    pts = np.round(np.stack([c[:, 0], size[1] - c[:, 1]], 1) * PPM).astype(np.int32)
    cv2.polylines(img, [pts], True, ROAD_BGR, int(width_m * PPM), cv2.LINE_AA)
    return img, c, size

def nearest_on_centre(centre, p):
    d = np.linalg.norm(centre - p, axis=1)
    i = int(d.argmin())
    return d[i], i

# ----------------------------------------------------------------------------- camera
class Camera:
    def __init__(self, w, h, hfov_deg, height_m, pitch_deg):
        self.w, self.h = w, h
        self.fx = self.fy = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
        self.cx, self.cy = w / 2, h / 2
        self.hc, th = height_m, math.radians(pitch_deg)
        K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.]])
        # car frame (X fwd, Y left) ground point -> camera coords, see docstring in repo wiki
        E = np.array([[0, -1, 0], [-math.sin(th), 0, height_m * math.cos(th)],
                      [math.cos(th), 0, height_m * math.sin(th)]])
        self.H_car = K @ E
        self.horizon = int(self.cy - self.fy * math.tan(th)) + 2

    def render(self, world, size_m, x, y, psi):
        # map pixel -> world metres
        M = np.array([[1 / PPM, 0, 0], [0, -1 / PPM, size_m[1]], [0, 0, 1]])
        # world -> car frame
        c, s = math.cos(psi), math.sin(psi)
        R = np.array([[c, s, -(c * x + s * y)], [-s, c, (s * x - c * y)], [0, 0, 1]])
        H = self.H_car @ R @ M
        img = cv2.warpPerspective(world, H, (self.w, self.h), flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=FLOOR_BGR)
        img[:max(self.horizon, 0)] = WALL_BGR
        return img

# ----------------------------------------------------------------------------- reference perception
def ref_process_image(image, stamp):
    small = cv2.resize(image, (160, 120), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    # Yellow paper road measured on the real camera: H~26, S~170, V~228.
    # S/V floors keep the beige wall (low S) out; rows above the table are ignored.
    mask = cv2.inRange(hsv, (18, 120, 120), (35, 255, 255))
    mask[:50] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    band = mask[80:100]
    cols = np.where(band.any(axis=0))[0]
    if len(cols) < 8:
        return {'offset_px': 0.0, 'width_px': 0.0, 'valid': False, 'stamp': stamp}
    left, right = float(cols.min()), float(cols.max())
    return {'offset_px': (left + right) / 2 - 80.0, 'width_px': right - left, 'valid': True, 'stamp': stamp}

# ----------------------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--track', default='oval', choices=['oval', 'figure8', 'wiggle'])
    ap.add_argument('--road-width', type=float, default=0.30, help='metres')
    ap.add_argument('--speed', type=float, default=None, help='override requested_forward_speed_m_s')
    ap.add_argument('--limit', type=float, default=0.15, help='tread speed limit (Low 0.15/Med 0.60/High 1.40)')
    ap.add_argument('--track-width', type=float, default=0.20, help='track_width_m used by safety conversion')
    ap.add_argument('--latency', type=int, default=2, help='image latency in 50 ms ticks')
    ap.add_argument('--lag', type=float, default=0.15, help='motor first-order lag time constant (s)')
    ap.add_argument('--start-offset', type=float, default=0.08, help='initial lateral error (m)')
    ap.add_argument('--cam', default='640x360', help='rendered camera size')
    ap.add_argument('--hfov', type=float, default=70.0)
    ap.add_argument('--cam-height', type=float, default=0.15)
    ap.add_argument('--cam-pitch', type=float, default=15.0, help='degrees downward')
    ap.add_argument('--perception', default='auto', choices=['auto', 'node', 'ref'])
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE', help='override a yaml parameter')
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--video', type=Path)
    ap.add_argument('--plot', type=Path)
    ap.add_argument('--csv', type=Path)
    ap.add_argument('--nav', type=Path, default=NAV_DIR / 'navigation.py')
    a = ap.parse_args()

    params = load_yaml_params(YAML)
    for kv in a.set:
        k, v = kv.split('=', 1); params[k] = float(v)
    if a.speed is not None:
        params['requested_forward_speed_m_s'] = a.speed
    params.setdefault('debug_fixed_offset_px', float('nan'))

    install_fake_ros()
    FakeNode.params = params
    import importlib.util
    spec = importlib.util.spec_from_file_location('navigation', a.nav)
    navmod = importlib.util.module_from_spec(spec); spec.loader.exec_module(navmod)
    nav = navmod.Navigation()

    world, centre, size_m = make_track(a.track, a.road_width)
    cw, ch = (int(s) for s in a.cam.split('x'))
    cam = Camera(cw, ch, a.hfov, a.cam_height, a.cam_pitch)

    # start on the centreline, facing along it, displaced sideways
    i0 = 0
    heading = math.atan2(*(centre[i0 + 1] - centre[i0])[::-1])
    x, y = centre[i0] + a.start_offset * np.array([-math.sin(heading), math.cos(heading)])
    psi = heading
    v_act = w_act = 0.0
    dt = 0.05
    frames = deque()
    log, trail = [], []
    used_ref = False
    writer = None
    if a.video:
        writer = cv2.VideoWriter(str(a.video), cv2.VideoWriter_fourcc(*'mp4v'), 20, (cw + 320, max(ch, 320)))
    window = not a.headless
    if window:
        cv2.namedWindow('sim', cv2.WINDOW_NORMAL)
    t0 = time.monotonic()
    steps = int(a.seconds / dt)
    off_track_steps = 0
    for k in range(steps):
        CLOCK.t = k * dt
        frames.append((cam.render(world, size_m, x, y, psi), CLOCK.now()))
        if len(frames) <= a.latency:
            continue
        image, stamp = frames.popleft()
        nav.image, nav.image_stamp = image, stamp

        # ---- the real pipeline ----
        obs = nav.process_image(image) if a.perception != 'ref' else None
        if obs is None and a.perception != 'node':
            obs = ref_process_image(image, stamp)
            if not used_ref:
                used_ref = True
                print('note: Navigation.process_image returned None; using the reference detector', file=sys.stderr)
        est = nav.update_estimate(obs, None)
        act = nav.choose_action(est)
        v_req, w_req = nav.compute_motion(act)
        if not all(math.isfinite(q) for q in (v_req, w_req)):
            v_req = w_req = 0.0

        # ---- safety conversion + speed profile (as in the base) ----
        tl, tr = v_req - a.track_width * w_req / 2, v_req + a.track_width * w_req / 2
        m = max(abs(tl), abs(tr))
        if m > a.limit:
            tl, tr = tl * a.limit / m, tr * a.limit / m
        v_cmd, w_cmd = (tl + tr) / 2, (tr - tl) / a.track_width

        # ---- car: first-order actuator lag + unicycle ----
        alpha = dt / max(a.lag, dt)
        v_act += alpha * (v_cmd - v_act); w_act += alpha * (w_cmd - w_act)
        x += v_act * math.cos(psi) * dt; y += v_act * math.sin(psi) * dt; psi += w_act * dt

        cte, ci = nearest_on_centre(centre, np.array([x, y]))
        if cte > a.road_width / 2 + 0.05:
            off_track_steps += 1
        trail.append((x, y))
        log.append(dict(t=CLOCK.t, offset_px=(obs or {}).get('offset_px', float('nan')),
                        valid=int(bool((obs or {}).get('valid'))), est_px=est['offset_px'] if est else float('nan'),
                        action=(act or {}).get('kind', 'none'), v_req=v_req, yaw_req=w_req,
                        v_cmd=v_cmd, yaw_cmd=w_cmd, cte_m=cte, x=x, y=y, psi=psi))

        if window or writer:
            canvas = draw(image, obs, est, act, v_req, w_req, v_cmd, w_cmd, cte, world, size_m, trail, centre, ci, cw, ch)
            if writer: writer.write(canvas)
            if window:
                cv2.imshow('sim', canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')): break
                # keep roughly real time
                lag = (t0 + CLOCK.t) - time.monotonic()
                if lag > 0: time.sleep(lag)
    if writer: writer.release()
    if window: cv2.destroyAllWindows()

    # ---- summary ----
    cte = np.array([r['cte_m'] for r in log])
    print(f'{len(log)} ticks, {CLOCK.t:.1f} s simulated; track={a.track} speed={params["requested_forward_speed_m_s"]} '
          f'latency={a.latency*50} ms')
    print(f'cross-track error: mean {cte.mean()*100:.1f} cm, max {cte.max()*100:.1f} cm, '
          f'off-road {off_track_steps*dt:.1f} s, stops {sum(r["action"]!="follow" for r in log)*dt:.1f} s')
    if a.csv:
        with a.csv.open('w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=list(log[0])); wr.writeheader(); wr.writerows(log)
        print('csv ->', a.csv)
    if a.plot:
        try:
            import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        except ImportError:
            print('matplotlib not installed; skipping plot'); return
        t = [r['t'] for r in log]
        fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        ax[0].plot(t, [r['offset_px'] for r in log], lw=.8, label='measured offset (px)')
        ax[0].plot(t, [r['est_px'] for r in log], lw=1.2, label='estimate (px)'); ax[0].axhline(0, c='k', lw=.5)
        ax[0].legend(); ax[0].set_ylabel('offset [px]')
        ax[1].plot(t, [r['yaw_req'] for r in log], label='yaw request'); ax[1].plot(t, [r['v_req'] for r in log], label='v request')
        ax[1].legend(); ax[1].set_ylabel('rad/s, m/s')
        ax[2].plot(t, cte * 100); ax[2].axhline(a.road_width * 50, c='r', ls='--', lw=.8)
        ax[2].set_ylabel('cross-track [cm]'); ax[2].set_xlabel('t [s]')
        fig.suptitle(f'{a.track}, v={params["requested_forward_speed_m_s"]} m/s, latency {a.latency*50} ms, '
                     f'kp={params.get("kp_rad_s_per_px")} kd={params.get("kd_rad_per_px")}')
        fig.tight_layout(); fig.savefig(a.plot, dpi=120); print('plot ->', a.plot)

def draw(image, obs, est, act, v_req, w_req, v_cmd, w_cmd, cte, world, size_m, trail, centre, ci, cw, ch):
    img = image.copy()
    sx, sy = cw / 160, ch / 120
    cv2.rectangle(img, (0, int(80 * sy)), (cw - 1, int(100 * sy)), (255, 255, 255), 1)
    cv2.line(img, (cw // 2, int(80 * sy)), (cw // 2, int(100 * sy)), (255, 255, 255), 1)
    if obs and obs.get('valid'):
        u = int((80 + obs['offset_px']) * sx)
        cv2.line(img, (u, int(78 * sy)), (u, int(102 * sy)), (0, 0, 255), 2)
    if est:
        u = int((80 + est['offset_px']) * sx)
        cv2.circle(img, (u, int(90 * sy)), 5, (255, 0, 0), -1)
    kind = (act or {}).get('kind', 'none')
    txt = [f'{kind}  obs={obs["offset_px"]:+.0f}px' if obs and obs.get('valid') else f'{kind}  obs=none',
           f'req v={v_req:.3f} yaw={w_req:+.2f}', f'cmd v={v_cmd:.3f} yaw={w_cmd:+.2f}', f'cte={cte*100:.1f}cm']
    for i, s in enumerate(txt):
        cv2.putText(img, s, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
        cv2.putText(img, s, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    # minimap
    mm = cv2.resize(world, (320, int(320 * size_m[1] / size_m[0])))
    k = 320 / world.shape[1]
    def to_mm(p): return (int(p[0] * PPM * k), int((size_m[1] - p[1]) * PPM * k))
    for i in range(1, len(trail)):
        cv2.line(mm, to_mm(trail[i - 1]), to_mm(trail[i]), (0, 0, 255), 1)
    cv2.circle(mm, to_mm(trail[-1]), 4, (255, 0, 0), -1)
    H = max(ch, 320)
    canvas = np.full((H, cw + 320, 3), 30, np.uint8)
    canvas[:ch, :cw] = img
    canvas[:mm.shape[0], cw:cw + 320] = mm
    return canvas

if __name__ == '__main__':
    main()
