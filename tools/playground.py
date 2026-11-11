#!/usr/bin/env python3
"""Playground for team_navigation: simulated map + camera, closed-loop runs,
replay of real frames, interactive tuning, parameter sweeps. No ROS needed.

It imports the REAL navigation.py (any version: the Youran px-contract one or
Suhavi's fraction-contract one), reads the REAL navigation.yaml, and calls the
four hooks exactly like tick() does on the robot.

Quick start (from the repo root, `pip install opencv-python numpy matplotlib`):

    python tools/playground.py                              # window, oval track
    python tools/playground.py --track fork --tiles --blue   # paper tiles, blue borders, a fork
    python tools/playground.py --map tools/maps/fork.png --start 1.0,0.8,0 --ppm 100
    python tools/playground.py --replay ~/frames             # real camera frames, open loop
    python tools/playground.py --headless --seconds 60 --plot out.png --video out.mp4
    python tools/playground.py --sweep requested_forward_speed_m_s=0.05,0.15,0.3
    python tools/playground.py --nav ~/suhavi_wip/navigation.py --yaml ~/suhavi_wip/navigation.yaml
    python tools/playground.py --make-maps tools/maps        # write example PNG maps

Keys in the window: space pause, n single step, r reset, arrows nudge the car
(left/right = sideways, up/down = heading), m toggle the node's debug image,
+ / - speed, [ / ] first --tune parameter, s screenshot, q quit.

Model: ideal pinhole camera (height, pitch, HFOV configurable), optional
noise / exposure drift / JPEG round trip; unicycle car with first-order
actuator lag; image latency in ticks; the base's tread conversion and speed
profile clamp. No slip, no real motor dynamics — use it for signs, gains,
latency tolerance, fork logic and the handout's offset-over-time plots, then
confirm on the car.
"""
import argparse
import csv
import importlib.util
import json
import math
import random
import sys
import time
import types
from collections import deque
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NAV_DEFAULT = ROOT / 'robot' / 'src' / 'team_navigation' / 'team_navigation' / 'navigation.py'
YAML_DEFAULT = ROOT / 'robot' / 'src' / 'team_navigation' / 'config' / 'navigation.yaml'
DT = 0.05
PPM_DEFAULT = 100
# colours measured from the real camera (yellow paper on a grey table, blue paper borders)
ROAD_BGR, FLOOR_BGR, BLUE_BGR, GAP_BGR, WALL_BGR = (60, 200, 235), (118, 118, 118), (150, 70, 40), (120, 120, 120), (190, 200, 205)


# ============================================================================ fake ROS
class SimTime:
    def __init__(self, ns): self.nanoseconds = int(ns)
    def __sub__(self, o): return SimTime(self.nanoseconds - o.nanoseconds)
    def to_msg(self): return self
    @staticmethod
    def from_msg(m): return m
    @property
    def sec(self): return self.nanoseconds // 1_000_000_000
    @property
    def nanosec(self): return self.nanoseconds % 1_000_000_000


class SimClock:
    t = 0.0
    def now(self): return SimTime(self.t * 1e9)


CLOCK = SimClock()


class _Param:
    def __init__(self, v): self.value = v


class _Logger:
    quiet = False
    def _p(self, lvl, m):
        if not self.quiet: print(f'[{lvl}] {m}', file=sys.stderr)
    def info(self, m): self._p('INFO', m)
    def warn(self, m): self._p('WARN', m)
    def error(self, m): self._p('ERROR', m)
    def debug(self, m): pass


class _Pub:
    """Captures what the node publishes. CompressedImage data is decoded for display."""
    captured = {}
    def __init__(self, topic): self.topic = topic
    def publish(self, m): _Pub.captured[self.topic] = m
    def get_subscription_count(self): return 1


class FakeNode:
    params = {}
    def __init__(self, name): self._name = name
    def declare_parameter(self, name, default=None):
        return _Param(self.params.get(name, default))
    def create_subscription(self, *a, **k): return None
    def create_publisher(self, msg_type, topic, *a, **k): return _Pub(topic)
    def create_timer(self, *a, **k): return None
    def get_clock(self): return CLOCK
    def get_logger(self): return _Logger()
    def destroy_node(self): pass


class _Msg(types.SimpleNamespace):
    """Stands in for CompressedImage / MotionRequest: attribute bag with a header."""
    def __init__(self, **kw):
        super().__init__(**kw)
        if not hasattr(self, 'header'): self.header = types.SimpleNamespace(stamp=None, frame_id='')


def install_fake_ros():
    def mod(name, **attrs):
        m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
    mod('rclpy', init=lambda *a, **k: None, spin=lambda n: None, ok=lambda: False, shutdown=lambda: None)
    mod('rclpy.node', Node=FakeNode)
    mod('rclpy.qos', qos_profile_sensor_data=None, QoSProfile=lambda **k: None,
        ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1))
    mod('rclpy.executors', ExternalShutdownException=Exception)
    mod('rclpy.time', Time=SimTime)
    mod('sensor_msgs'); mod('sensor_msgs.msg', Image=object, Imu=object, CompressedImage=_Msg)
    mod('std_msgs'); mod('std_msgs.msg', String=_Msg, Header=_Msg)
    mod('cv_bridge', CvBridge=lambda: None)
    mod('course_robot_interfaces'); mod('course_robot_interfaces.msg', MotionRequest=_Msg)


def load_yaml_params(path):
    """Flat `key: value` parser for navigation.yaml (lists like [18, 80, 80] supported). No PyYAML needed."""
    params = {}
    for line in Path(path).read_text().splitlines():
        line = line.split('#', 1)[0].rstrip()
        if ':' not in line or line.strip().endswith(':'):
            continue
        k, v = (s.strip() for s in line.strip().split(':', 1))
        params[k] = parse_scalar(v)
    return params


def parse_scalar(v):
    if v.startswith('[') and v.endswith(']'):
        return [parse_scalar(x.strip()) for x in v[1:-1].split(',') if x.strip()]
    if v in ('.nan', 'nan', '.NaN'): return float('nan')
    if v.lower() in ('true', 'false'): return v.lower() == 'true'
    try:
        return int(v)
    except ValueError:
        try: return float(v)
        except ValueError: return v.strip('"\'')


def load_navigation(nav_path, params):
    install_fake_ros()
    FakeNode.params = params
    spec = importlib.util.spec_from_file_location('navigation', nav_path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod.Navigation()


# ============================================================================ world
class World:
    """Map image (BGR, PPM px/m, y up in metres), centrelines for metrics, start pose."""
    def __init__(self, img, ppm, centres, start, name):
        self.img, self.ppm, self.centres, self.start, self.name = img, ppm, centres, start, name
        self.size_m = np.array([img.shape[1] / ppm, img.shape[0] / ppm])
        road = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (15, 90, 90), (40, 255, 255))
        k = max(3, int(0.03 * ppm)) | 1
        road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))   # paper seams are still road
        self.margin_px = cv2.distanceTransform(road, cv2.DIST_L2, 3)  # distance to road edge

    def to_px(self, p):
        return int(p[0] * self.ppm), int((self.size_m[1] - p[1]) * self.ppm)

    def margin_m(self, p):
        u, v = self.to_px(p)
        if 0 <= v < self.margin_px.shape[0] and 0 <= u < self.margin_px.shape[1]:
            return float(self.margin_px[v, u]) / self.ppm
        return 0.0

    def cte(self, p, current=-1):
        """Cross-track error to the followed centreline and its index (nan, -1 for PNG maps).

        The branch only changes when another centreline is closer by 10 cm, so
        'branch_switches' counts real lane changes, not jitter where branches run side by side."""
        if not self.centres:
            return float('nan'), -1
        d = [float(np.linalg.norm(c - p, axis=1).min()) for c in self.centres]
        best = int(np.argmin(d))
        if current >= 0 and d[current] <= d[best] + 0.10:
            best = current
        return d[best], best


def _loop(kind):
    t = np.linspace(0, 2 * np.pi, 500, endpoint=False)
    if kind == 'oval':
        return [np.stack([2.0 * np.cos(t), 1.2 * np.sin(t)], 1)]
    if kind == 'figure8':
        return [np.stack([2.2 * np.sin(t), 1.1 * np.sin(2 * t)], 1)]
    if kind == 'wiggle':
        r = 1.6 + 0.35 * np.sin(3 * t) + 0.15 * np.cos(5 * t)
        return [np.stack([r * np.cos(t), r * np.sin(t)], 1)]
    if kind == 'fork':
        # oval main loop + an inner chord that leaves the loop at the top and rejoins at the bottom.
        # oval + a Y-fork: a branch peels off tangentially on the right-hand end, runs ~45 cm
        # outside the main road, and rejoins tangentially (both branches are valid road).
        main = np.stack([2.0 * np.cos(t), 1.2 * np.sin(t)], 1)
        t0, t1 = math.radians(-70), math.radians(70)
        tb = np.linspace(t0, t1, 160)
        base = np.stack([2.0 * np.cos(tb), 1.2 * np.sin(tb)], 1)
        tang = np.stack([-2.0 * np.sin(tb), 1.2 * np.cos(tb)], 1); tang /= np.linalg.norm(tang, axis=1, keepdims=True)
        normal = np.stack([tang[:, 1], -tang[:, 0]], 1)           # outward on the right-hand side
        bump = 0.45 * np.sin(np.pi * (tb - t0) / (t1 - t0)) ** 2
        return [main, base + bump[:, None] * normal]
    if kind == 'straight':
        x = np.linspace(0, 6, 300)
        return [np.stack([x, np.zeros_like(x)], 1)]
    raise SystemExit(f'unknown track {kind}')


def make_world(kind, road_w, ppm, tiles, blue, seed=0):
    rng = random.Random(seed)
    centres = _loop(kind)
    allpts = np.concatenate(centres)
    shift = allpts.min(0) - 0.9
    centres = [c - shift for c in centres]
    size = np.concatenate(centres).max(0) + 0.9
    img = np.full((int(size[1] * ppm), int(size[0] * ppm), 3), FLOOR_BGR, np.uint8)
    # floor texture
    img = cv2.add(img, rng_noise(img.shape, 4, seed))
    def poly(c, closed):
        return np.round(np.stack([c[:, 0], size[1] - c[:, 1]], 1) * ppm).astype(np.int32)
    closed = kind != 'straight'
    for i, c in enumerate(centres):      # all blue first, then all road, so junctions stay yellow
        cl = closed and i == 0
        if blue:
            for sgn in (-1, 1):
                off = offset_curve(c, sgn * (road_w / 2 + 0.02 + 0.11), cl)
                # blue paper sheets with random gaps, like the real course
                n = len(off); seg = max(8, n // 24); j = 0
                while j < n:
                    L = seg + rng.randint(-2, 2)
                    if rng.random() > 0.15:
                        cv2.polylines(img, [poly(off[j:j + L], False)], False, BLUE_BGR, int(0.22 * ppm), cv2.LINE_AA)
                    j += L + rng.randint(0, 3)
    for i, c in enumerate(centres):
        cl = closed and i == 0
        cv2.polylines(img, [poly(c, cl)], cl, ROAD_BGR, int(road_w * ppm), cv2.LINE_AA)
    if tiles:
        # letter-size sheets ~0.28 m: thin grey seams across the road every sheet
        for i, c in enumerate(centres):
            cl = closed and i == 0
            d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))]
            for sdist in np.arange(0, d[-1], 0.28):
                k = int(np.searchsorted(d, sdist)); k = min(k, len(c) - 2)
                tangent = c[k + 1] - c[k]; tangent /= np.linalg.norm(tangent) + 1e-9
                normal = np.array([-tangent[1], tangent[0]])
                a, b = c[k] - normal * road_w / 2, c[k] + normal * road_w / 2
                cv2.line(img, poly(a[None], False)[0], poly(b[None], False)[0], GAP_BGR, max(1, int(0.006 * ppm)), cv2.LINE_AA)
    c0 = centres[0]
    i0 = len(c0) // 2 if kind == 'fork' else 0        # fork: start on the far side, reach the fork after half a lap
    heading = math.atan2(*(c0[i0 + 1] - c0[i0])[::-1])
    return World(img, ppm, centres, (float(c0[i0][0]), float(c0[i0][1]), heading), kind)


def offset_curve(c, d, closed):
    nxt = np.roll(c, -1, 0) if closed else np.r_[c[1:], c[-1:]]
    prv = np.roll(c, 1, 0) if closed else np.r_[c[:1], c[:-1]]
    t = nxt - prv; t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-9
    n = np.stack([-t[:, 1], t[:, 0]], 1)
    return c + d * n


def rng_noise(shape, sigma, seed):
    g = np.random.default_rng(seed)
    return np.clip(g.normal(0, sigma, shape), 0, 255).astype(np.uint8)


def load_png_world(path, ppm, start):
    img = cv2.imread(str(path))
    if img is None: raise SystemExit(f'cannot read {path}')
    if start is None: raise SystemExit('--map needs --start x,y,heading_deg (metres, y up from the bottom-left)')
    x, y, h = (float(s) for s in start.split(','))
    return World(img, ppm, [], (x, y, math.radians(h)), Path(path).stem)


def write_example_maps(d):
    d = Path(d); d.mkdir(parents=True, exist_ok=True)
    for kind in ('oval', 'figure8', 'wiggle', 'fork', 'straight'):
        w = make_world(kind, 0.30, PPM_DEFAULT, tiles=True, blue=True)
        cv2.imwrite(str(d / f'{kind}.png'), w.img)
        meta = dict(ppm=w.ppm, start=[round(w.start[0], 3), round(w.start[1], 3), round(math.degrees(w.start[2]), 1)],
                    note='start = x,y in metres from the bottom-left corner, heading in degrees CCW from +x')
        (d / f'{kind}.json').write_text(json.dumps(meta, indent=1))
        print(f'{d / (kind + ".png")}  --ppm {w.ppm} --start {meta["start"][0]},{meta["start"][1]},{meta["start"][2]}')


# ============================================================================ camera
class Camera:
    def __init__(self, w, h, hfov_deg, height_m, pitch_deg, noise=0.0, jpeg=0, exposure=0.0, seed=0):
        self.w, self.h = w, h
        self.fx = self.fy = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
        self.cx, self.cy = w / 2, h / 2
        th = math.radians(pitch_deg)
        K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.]])
        # car-frame ground point (X fwd, Y left, 1) -> camera pixel
        E = np.array([[0, -1, 0], [-math.sin(th), 0, height_m * math.cos(th)],
                      [math.cos(th), 0, height_m * math.sin(th)]])
        self.H_car = K @ E
        self.horizon = int(self.cy - self.fy * math.tan(th)) + 2
        self.noise, self.jpeg, self.exposure = noise, jpeg, exposure
        self.rng = np.random.default_rng(seed)
        self.wall = self._wall()

    def _wall(self):
        """A plain wall with a darker band, so the top of the image is not uniform."""
        wall = np.full((self.h, self.w, 3), WALL_BGR, np.uint8)
        cv2.rectangle(wall, (0, 0), (self.w, max(self.horizon // 3, 1)), (160, 170, 175), -1)
        return wall

    def render(self, world, x, y, psi):
        M = np.array([[1 / world.ppm, 0, 0], [0, -1 / world.ppm, world.size_m[1]], [0, 0, 1]])
        c, s = math.cos(psi), math.sin(psi)
        R = np.array([[c, s, -(c * x + s * y)], [-s, c, (s * x - c * y)], [0, 0, 1]])
        H = self.H_car @ R @ M
        img = cv2.warpPerspective(world.img, H, (self.w, self.h), flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=FLOOR_BGR)
        hz = max(self.horizon, 0)
        img[:hz] = self.wall[:hz]
        if self.exposure:
            img = cv2.convertScaleAbs(img, alpha=1 + self.rng.uniform(-self.exposure, self.exposure))
        if self.noise:
            img = np.clip(img.astype(np.int16) + self.rng.normal(0, self.noise, img.shape), 0, 255).astype(np.uint8)
        if self.jpeg:
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg)])
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img


# ============================================================================ contracts
def obs_px(d):
    """Lateral offset in 160-px-image pixels from either contract; nan when invalid/missing."""
    if not d or d.get('valid') is False:
        return float('nan')
    if d.get('offset_px') is not None: return float(d['offset_px'])
    if d.get('offset') is not None: return float(d['offset']) * 80.0
    return float('nan')


def action_kind(act, est):
    if act is None: return 'stop'
    k = act.get('kind', 'follow')
    if (est or {}).get('fork') or act.get('fork'): k += '+fork'
    return k


def safety(v_req, w_req, track_width, limit):
    """The base's conversion: treads = v -/+ b*w/2, scaled together to the Y speed profile."""
    tl, tr = v_req - track_width * w_req / 2, v_req + track_width * w_req / 2
    m = max(abs(tl), abs(tr))
    if m > limit:
        tl, tr = tl * limit / m, tr * limit / m
    return (tl + tr) / 2, (tr - tl) / track_width


# ============================================================================ runner
class Runner:
    def __init__(self, a, params, world, cam, nav):
        self.a, self.params, self.world, self.cam, self.nav = a, params, world, cam, nav
        self.reset()

    def reset(self):
        a, w = self.a, self.world
        x, y, psi = w.start
        self.x = x - a.start_offset * math.sin(psi); self.y = y + a.start_offset * math.cos(psi)
        self.psi = psi + math.radians(a.start_heading)
        self.v_act = self.w_act = 0.0
        self.frames = deque(); self.log = []; self.trail = []
        self.k = 0; self.off_road = 0; self.used_ref = False
        CLOCK.t = 0.0
        # fresh node state without re-importing: re-run __init__ (params may have changed)
        FakeNode.params = self.params
        self.nav.__init__()
        _Pub.captured.clear()

    def step(self):
        a, w = self.a, self.world
        CLOCK.t = self.k * DT
        self.frames.append((self.cam.render(w, self.x, self.y, self.psi), CLOCK.now()))
        self.k += 1
        if len(self.frames) <= a.latency:
            return None
        image, stamp = self.frames.popleft()
        rec = self.run_pipeline(image, stamp)
        v_cmd, w_cmd = safety(rec['v_req'], rec['yaw_req'], a.track_width, a.limit)
        alpha = DT / max(a.lag, DT)
        self.v_act += alpha * (v_cmd - self.v_act); self.w_act += alpha * (w_cmd - self.w_act)
        wn = np.random.normal(0, a.process_noise) if a.process_noise else 0.0
        self.x += self.v_act * math.cos(self.psi) * DT
        self.y += self.v_act * math.sin(self.psi) * DT
        self.psi += (self.w_act + wn) * DT
        cte, branch = w.cte(np.array([self.x, self.y]), self.log[-1]['branch'] if self.log else -1)
        margin = w.margin_m((self.x, self.y))
        if margin <= 0: self.off_road += 1
        self.trail.append((self.x, self.y))
        rec.update(v_cmd=v_cmd, yaw_cmd=w_cmd, cte_m=cte, branch=branch, margin_m=margin, x=self.x, y=self.y, psi=self.psi)
        self.log.append(rec)
        return image, rec

    def run_pipeline(self, image, stamp):
        a, nav = self.a, self.nav
        nav.image, nav.image_stamp = image, stamp
        t0 = time.perf_counter()
        obs = nav.process_image(image) if a.perception != 'ref' else None
        if math.isnan(obs_px(obs)) and obs is not None and a.perception == 'auto' and not getattr(self, 'node_worked', False):
            obs = None
        if obs is None and a.perception != 'node':
            obs = ref_process_image(image, stamp)
            if not self.used_ref:
                self.used_ref = True
                print('note: node perception gave nothing; using the reference detector (--perception node to forbid)', file=sys.stderr)
        elif obs is not None and not math.isnan(obs_px(obs)):
            self.node_worked = True
        est = nav.update_estimate(obs, None)
        act = nav.choose_action(est)
        v_req, w_req = nav.compute_motion(act)
        ms = (time.perf_counter() - t0) * 1000
        if not all(math.isfinite(q) for q in (v_req, w_req)):
            v_req = w_req = 0.0
        return dict(t=CLOCK.t, offset_px=obs_px(obs), est_px=obs_px(est), action=action_kind(act, est),
                    v_req=v_req, yaw_req=w_req, pipeline_ms=ms)

    def summary(self):
        if not self.log: return {}
        cte = np.array([r['cte_m'] for r in self.log]); margin = np.array([r['margin_m'] for r in self.log])
        ms = np.array([r['pipeline_ms'] for r in self.log])
        br = [r['branch'] for r in self.log]
        switches = sum(1 for i in range(1, len(br)) if br[i] != br[i - 1] and -1 not in (br[i], br[i - 1]))
        return dict(track=self.world.name, seconds=round(CLOCK.t, 1), ticks=len(self.log),
                    cte_mean_cm=float(np.nanmean(cte) * 100), cte_max_cm=float(np.nanmax(cte) * 100),
                    margin_min_cm=float(margin.min() * 100), off_road_s=self.off_road * DT,
                    stop_s=sum(not r['action'].startswith('follow') for r in self.log) * DT,
                    fork_flag_s=sum('fork' in r['action'] for r in self.log) * DT, branch_switches=switches,
                    distance_m=float(sum(math.hypot(*(np.subtract(self.trail[i], self.trail[i - 1]))) for i in range(1, len(self.trail)))),
                    pipeline_ms_mean=float(ms.mean()), pipeline_ms_max=float(ms.max()))


def ref_process_image(image, stamp):
    """Reference detector, thresholds measured on a real frame (H~26,S~170,V~228 yellow paper)."""
    small = cv2.resize(image, (160, 120), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (18, 120, 120), (35, 255, 255)); mask[:50] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cols = np.where(mask[80:100].any(axis=0))[0]
    if len(cols) < 8:
        return {'offset_px': 0.0, 'width_px': 0.0, 'valid': False, 'stamp': stamp}
    return {'offset_px': (cols.min() + cols.max()) / 2 - 80.0, 'width_px': float(cols.max() - cols.min()), 'valid': True, 'stamp': stamp}


# ============================================================================ drawing
def debug_image():
    m = _Pub.captured.get('navigation/debug/compressed')
    if m is None or not getattr(m, 'data', None): return None
    return cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)


def draw_frame(image, rec, world, trail, show_debug, note=''):
    cw, ch = image.shape[1], image.shape[0]
    img = image.copy()
    sx, sy = cw / 160, ch / 120
    cv2.rectangle(img, (0, int(80 * sy)), (cw - 1, int(100 * sy)), (255, 255, 255), 1)
    cv2.line(img, (cw // 2, int(80 * sy)), (cw // 2, int(100 * sy)), (255, 255, 255), 1)
    if not math.isnan(rec['offset_px']):
        u = int((80 + rec['offset_px']) * sx); cv2.line(img, (u, int(78 * sy)), (u, int(102 * sy)), (0, 0, 255), 2)
    if not math.isnan(rec['est_px']):
        u = int((80 + rec['est_px']) * sx); cv2.circle(img, (u, int(90 * sy)), 5, (255, 0, 0), -1)
    lines = [f"{rec['action']}   obs={rec['offset_px']:+.0f}px" if not math.isnan(rec['offset_px']) else f"{rec['action']}   obs=none",
             f"req v={rec['v_req']:.3f} yaw={rec['yaw_req']:+.2f}   cmd v={rec.get('v_cmd', 0):.3f} yaw={rec.get('yaw_cmd', 0):+.2f}",
             f"t={rec['t']:.1f}s  pipeline {rec['pipeline_ms']:.1f} ms" + (f"  cte={rec['cte_m']*100:.1f}cm" if not math.isnan(rec.get('cte_m', float('nan'))) else '') + (f"  margin={rec.get('margin_m', 0)*100:.0f}cm"),
             note]
    for i, s in enumerate(lines):
        cv2.putText(img, s, (8, 20 + 19 * i), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
        cv2.putText(img, s, (8, 20 + 19 * i), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    side_w = 360
    panel = np.full((ch, side_w, 3), 30, np.uint8)
    if world is not None:
        mm_h = int(side_w * world.size_m[1] / world.size_m[0]); mm_h = min(mm_h, ch // 2)
        mm_w = int(mm_h * world.size_m[0] / world.size_m[1])
        mm = cv2.resize(world.img, (mm_w, mm_h))
        kx, ky = mm_w / world.img.shape[1], mm_h / world.img.shape[0]
        def tp(p):
            u, v = world.to_px(p); return int(u * kx), int(v * ky)
        for i in range(1, len(trail)):
            cv2.line(mm, tp(trail[i - 1]), tp(trail[i]), (0, 0, 255), 1)
        if trail:
            cv2.circle(mm, tp(trail[-1]), 4, (255, 0, 0), -1)
        panel[:mm_h, :mm_w] = mm
        y0 = mm_h + 4
    else:
        y0 = 4
    dbg = debug_image() if show_debug else None
    if dbg is not None:
        h = min(ch - y0, int(side_w * dbg.shape[0] / dbg.shape[1]))
        if h > 10:
            panel[y0:y0 + h, :side_w] = cv2.resize(dbg, (side_w, h))
            cv2.putText(panel, 'navigation/debug', (4, y0 + 14), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
    return np.hstack([img, panel])


def plot(log, path, title):
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed; skipping plot'); return
    t = [r['t'] for r in log]
    has_cte = not all(math.isnan(r.get('cte_m', float('nan'))) for r in log)
    fig, ax = plt.subplots(3 if has_cte else 2, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(t, [r['offset_px'] for r in log], lw=.8, label='measured offset (px of 160)')
    ax[0].plot(t, [r['est_px'] for r in log], lw=1.2, label='estimate'); ax[0].axhline(0, c='k', lw=.5)
    stops = [r['t'] for r in log if not r['action'].startswith('follow')]
    if stops: ax[0].scatter(stops, [0] * len(stops), s=4, c='r', label='stop')
    ax[0].legend(loc='upper right'); ax[0].set_ylabel('offset [px]')
    ax[1].plot(t, [r['yaw_req'] for r in log], label='yaw request [rad/s]'); ax[1].plot(t, [r['v_req'] for r in log], label='v request [m/s]')
    ax[1].legend(loc='upper right')
    if has_cte:
        ax[2].plot(t, [r['cte_m'] * 100 for r in log]); ax[2].set_ylabel('cross-track [cm]')
    ax[-1].set_xlabel('t [s]'); fig.suptitle(title); fig.tight_layout(); fig.savefig(path, dpi=120)
    print('plot ->', path)


# ============================================================================ modes
def closed_loop(a, params, nav):
    world = load_png_world(a.map, a.ppm, a.start) if a.map else make_world(a.track, a.road_width, a.ppm, a.tiles, a.blue, a.seed)
    cw, ch = (int(s) for s in a.cam.split('x'))
    cam = Camera(cw, ch, a.hfov, a.cam_height, a.cam_pitch, a.noise, a.jpeg, a.exposure, a.seed)
    run = Runner(a, params, world, cam, nav)
    writer = cv2.VideoWriter(str(a.video), cv2.VideoWriter_fourcc(*'mp4v'), 20, (cw + 360, ch)) if a.video else None
    window = not a.headless
    if window: cv2.namedWindow('playground', cv2.WINDOW_NORMAL)
    paused, single, show_debug, note, canvas = False, False, True, '', None
    tune = a.tune[0] if a.tune else None
    t_wall = time.monotonic()
    steps = int(a.seconds / DT)
    while run.k < steps:
        if not paused or single:
            out = run.step(); single = False
            if out is not None and (window or writer):
                canvas = draw_frame(out[0], out[1], world, run.trail, show_debug, note)
                if writer: writer.write(canvas)
                if window: cv2.imshow('playground', canvas)
        if window:
            key = cv2.waitKey(1 if not paused else 30) & 0xFF
            if key in (27, ord('q')): break
            elif key == ord(' '): paused = not paused
            elif key == ord('n'): single = True
            elif key == ord('r'): run.reset(); t_wall = time.monotonic()
            elif key == ord('m'): show_debug = not show_debug
            elif key == ord('s') and canvas is not None:
                p = Path(f'playground_{int(time.time())}.png'); cv2.imwrite(str(p), canvas); print('screenshot ->', p)
            elif key in (81, 83, 2, 3):  # left/right: nudge sideways 5 cm
                d = 0.05 if key in (81, 2) else -0.05
                run.x -= d * math.sin(run.psi); run.y += d * math.cos(run.psi)
            elif key in (82, 84, 0, 1):  # up/down: heading +-10 deg
                run.psi += math.radians(10 if key in (82, 0) else -10)
            elif key in (ord('+'), ord('=')) or key == ord('-'):
                k = 'requested_forward_speed_m_s'; params[k] = round(params.get(k, .05) * (1.25 if key != ord('-') else .8), 4)
                nav.__init__(); note = f'{k}={params[k]}'
            elif key in (ord('['), ord(']')) and tune:
                params[tune] = round(params.get(tune, 0) * (1.25 if key == ord(']') else .8), 5)
                nav.__init__(); note = f'{tune}={params[tune]}'
            if not paused:
                lag = (t_wall + CLOCK.t) - time.monotonic()
                if lag > 0: time.sleep(min(lag, DT))
    if writer: writer.release()
    if window: cv2.destroyAllWindows()
    return run


def replay(a, params, nav):
    src = Path(a.replay)
    files = sorted(p for p in src.iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png')) if src.is_dir() else None
    cap = None if files else cv2.VideoCapture(str(src))
    run = Runner.__new__(Runner); run.a, run.params, run.world, run.cam, run.nav = a, params, None, None, nav
    run.log, run.trail, run.used_ref, run.off_road = [], [], False, 0
    FakeNode.params = params; nav.__init__()
    window = not a.headless
    writer = None
    if window: cv2.namedWindow('playground', cv2.WINDOW_NORMAL)
    k = 0
    while True:
        if files:
            if k >= len(files): break
            image = cv2.imread(str(files[k])); name = files[k].name
        else:
            ok, image = cap.read(); name = f'frame {k}'
            if not ok: break
        CLOCK.t = k * DT + 0.03; stamp = SimTime((k * DT) * 1e9)   # 30 ms old, like the real camera
        rec = run.run_pipeline(image, stamp); rec['file'] = name; rec['cte_m'] = float('nan'); rec['margin_m'] = 0
        run.log.append(rec); k += 1
        if window or a.video:
            canvas = draw_frame(image, rec, None, [], True, name)
            if a.video:
                if writer is None:
                    writer = cv2.VideoWriter(str(a.video), cv2.VideoWriter_fourcc(*'mp4v'), 10, (canvas.shape[1], canvas.shape[0]))
                writer.write(canvas)
            if window:
                cv2.imshow('playground', canvas)
                key = cv2.waitKey(0 if a.step else 50) & 0xFF
                if key in (27, ord('q')): break
    if writer: writer.release()
    if window: cv2.destroyAllWindows()
    ms = np.array([r['pipeline_ms'] for r in run.log]); valid = sum(not math.isnan(r['offset_px']) for r in run.log)
    print(f'{len(run.log)} frames, road found in {valid} ({100*valid/max(1,len(run.log)):.0f}%), '
          f'pipeline {ms.mean():.1f} ms mean / {ms.max():.1f} ms max (budget 50 ms), '
          f'stops {sum(not r["action"].startswith("follow") for r in run.log)}, fork flags {sum("fork" in r["action"] for r in run.log)}')
    return run


def sweep(a, params, nav_path):
    key, vals = a.sweep.split('=', 1)
    rows = []
    _Logger.quiet = True
    for v in vals.split(','):
        p = dict(params); p[key] = parse_scalar(v)
        nav = load_navigation(nav_path, p)
        a2 = argparse.Namespace(**vars(a)); a2.headless = True; a2.video = None
        s = closed_loop(a2, p, nav).summary()
        rows.append((v, s))
        print(f"{key}={v:<8} cte mean {s['cte_mean_cm']:5.1f} cm  max {s['cte_max_cm']:5.1f}  off-road {s['off_road_s']:5.1f} s  "
              f"stops {s['stop_s']:5.1f} s  fork {s['fork_flag_s']:5.1f} s  switches {s['branch_switches']}  dist {s['distance_m']:.1f} m")
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_argument_group('code under test')
    g.add_argument('--nav', type=Path, default=NAV_DEFAULT, help='navigation.py to run')
    g.add_argument('--yaml', type=Path, default=YAML_DEFAULT, help='navigation.yaml to read')
    g.add_argument('--set', action='append', default=[], metavar='KEY=VALUE', help='override a yaml parameter')
    g.add_argument('--perception', default='auto', choices=['auto', 'node', 'ref'],
                   help='auto: node detector, reference fills in while it returns nothing')
    g = ap.add_argument_group('world')
    g.add_argument('--track', default='oval', choices=['oval', 'figure8', 'wiggle', 'fork', 'straight'])
    g.add_argument('--map', type=Path, help='PNG map instead of a built-in track (yellow road, blue borders)')
    g.add_argument('--ppm', type=int, default=PPM_DEFAULT, help='map pixels per metre')
    g.add_argument('--start', help='x,y,heading_deg for --map')
    g.add_argument('--road-width', type=float, default=0.30)
    g.add_argument('--tiles', action='store_true', help='paper-sheet seams across the road')
    g.add_argument('--blue', action='store_true', help='blue paper borders with gaps')
    g.add_argument('--seed', type=int, default=0)
    g = ap.add_argument_group('camera')
    g.add_argument('--cam', default='640x360', help='rendered size; the robot gives 1280x720')
    g.add_argument('--hfov', type=float, default=70.0)
    g.add_argument('--cam-height', type=float, default=0.15)
    g.add_argument('--cam-pitch', type=float, default=15.0, help='degrees downward')
    g.add_argument('--noise', type=float, default=0.0, help='gaussian pixel noise sigma, e.g. 6')
    g.add_argument('--jpeg', type=int, default=0, help='JPEG round trip quality, e.g. 70 (0 = off)')
    g.add_argument('--exposure', type=float, default=0.0, help='random brightness factor +- this, e.g. 0.15')
    g.add_argument('--latency', type=int, default=2, help='image latency in 50 ms ticks')
    g = ap.add_argument_group('car')
    g.add_argument('--limit', type=float, default=0.15, help='tread speed limit: Low 0.15 / Medium 0.60 / High 1.40')
    g.add_argument('--track-width', type=float, default=0.20, help='track_width_m for the tread conversion')
    g.add_argument('--lag', type=float, default=0.15, help='actuator time constant (s)')
    g.add_argument('--process-noise', type=float, default=0.0, help='yaw-rate noise sigma (rad/s)')
    g.add_argument('--start-offset', type=float, default=0.05, help='initial lateral error (m, + = left of centre)')
    g.add_argument('--start-heading', type=float, default=0.0, help='initial heading error (deg)')
    g = ap.add_argument_group('run')
    g.add_argument('--seconds', type=float, default=60)
    g.add_argument('--headless', action='store_true')
    g.add_argument('--tune', action='append', default=[], help='yaml key adjusted by [ and ] in the window')
    g.add_argument('--video', type=Path); g.add_argument('--plot', type=Path); g.add_argument('--csv', type=Path)
    g.add_argument('--json', type=Path, help='write the summary here')
    g.add_argument('--replay', help='folder of frames or a video: open-loop run of the pipeline')
    g.add_argument('--step', action='store_true', help='replay: wait for a key between frames')
    g.add_argument('--sweep', metavar='KEY=V1,V2,...', help='headless run per value, prints a table')
    g.add_argument('--make-maps', metavar='DIR', help='write example PNG maps and exit')
    a = ap.parse_args()

    if a.make_maps:
        write_example_maps(a.make_maps); return
    params = load_yaml_params(a.yaml)
    for kv in a.set:
        k, v = kv.split('=', 1); params[k] = parse_scalar(v)
    if a.sweep:
        sweep(a, params, a.nav); return
    nav = load_navigation(a.nav, params)
    if a.replay:
        run = replay(a, params, nav)
    else:
        run = closed_loop(a, params, nav)
        s = run.summary()
        print(json.dumps(s, indent=1) if a.json is None else s)
        if a.json: a.json.write_text(json.dumps(s, indent=1)); print('summary ->', a.json)
    if a.csv and run.log:
        with Path(a.csv).open('w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=list(run.log[0])); wr.writeheader(); wr.writerows(run.log)
        print('csv ->', a.csv)
    if a.plot and run.log:
        title = f"{a.nav.name}  {'replay ' + str(a.replay) if a.replay else (a.map or a.track)}  v={params.get('requested_forward_speed_m_s')}  latency {a.latency*50} ms"
        plot(run.log, a.plot, title)


if __name__ == '__main__':
    main()
