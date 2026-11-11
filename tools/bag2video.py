#!/usr/bin/env python3
"""Turn a bag recorded with tools/record.sh into a video, CSVs, plots and replay frames.

Runs on the laptop, no ROS needed:  pip install rosbags opencv-python numpy matplotlib

    python tools/bag2video.py ~/bags/20261112_150301_lap_low
    python tools/bag2video.py BAG --out runs/lap_low            # output prefix
    python tools/bag2video.py BAG --frames runs/lap_low_frames  # also dump JPEGs for playground --replay
    python tools/bag2video.py BAG --no-video                    # just CSV + plots (fast)

Outputs (<out> defaults to the bag folder name):
    <out>.mp4              camera with the node's numbers overlaid, debug image beside it when published
    <out>_telemetry.csv    one row per navigation tick (offset, estimate, action, image age, pipeline ms)
    <out>_request.csv      navigation/request (v, yaw)
    <out>_motors.csv       motors/command (what safety actually sent to the treads)
    <out>_imu.csv          gyro / accel
    <out>.png              offset-over-time, request vs command, delays, gyro yaw rate
"""
import argparse
import csv
import json
import sys
from bisect import bisect_right
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def typestore_with_repo_msgs():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_JAZZY)
    add = {}
    for pkg in ('course_robot_interfaces', 'a2r_robot_interfaces'):
        for f in sorted((ROOT / 'robot' / 'src' / pkg / 'msg').glob('*.msg')):
            add.update(get_types_from_msg(f.read_text(), f'{pkg}/msg/{f.stem}'))
    ts.register(add)
    return ts


class Series:
    """Time-sorted samples with nearest-before lookup."""
    def __init__(self): self.t, self.v = [], []
    def add(self, t, v): self.t.append(t); self.v.append(v)
    def at(self, t):
        i = bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else None
    def __len__(self): return len(self.t)


def read_bag(path):
    from rosbags.highlevel import AnyReader
    ts = typestore_with_repo_msgs()
    data = {k: Series() for k in ('image', 'debug', 'telemetry', 'request', 'motors', 'imu', 'safety', 'joy', 'hw')}
    with AnyReader([Path(path)], default_typestore=ts) as reader:
        t0 = reader.start_time
        for conn, t_ns, raw in reader.messages():
            t = (t_ns - t0) / 1e9
            topic = conn.topic
            try:
                msg = reader.deserialize(raw, conn.msgtype)
            except Exception as exc:
                print(f'skip {topic}: {exc}', file=sys.stderr); continue
            if topic.endswith('camera/image_raw/compressed'):
                data['image'].add(t, bytes(msg.data))
            elif topic.endswith('navigation/debug/compressed'):
                data['debug'].add(t, bytes(msg.data))
            elif topic.endswith('navigation/telemetry'):
                try: data['telemetry'].add(t, json.loads(msg.data))
                except json.JSONDecodeError: pass
            elif topic.endswith('navigation/request'):
                data['request'].add(t, (float(msg.forward_speed_m_s), float(msg.yaw_rate_rad_s)))
            elif topic.endswith('motors/command'):
                data['motors'].add(t, (float(msg.left_m_s), float(msg.right_m_s), bool(msg.enabled), str(msg.control_mode), int(msg.speed_profile)))
            elif topic.endswith('imu/data'):
                data['imu'].add(t, (float(msg.angular_velocity.z), float(msg.linear_acceleration.x), float(msg.linear_acceleration.y)))
            elif topic.endswith('safety/status'):
                data['safety'].add(t, str(msg.data))
            elif topic.endswith('joystick/state'):
                data['joy'].add(t, (bool(msg.connected), int(msg.speed_profile)))
            elif topic.endswith('hardware/status'):
                data['hw'].add(t, str(msg.reason))
    return data


def flat(d, prefix):
    return {f'{prefix}_{k}': v for k, v in (d or {}).items()}


def write_csvs(data, out):
    rows = []
    for t, m in zip(data['telemetry'].t, data['telemetry'].v):
        r = {'t': t, 'image_age_ms': m.get('image_age_ms'), 'pipeline_ms': m.get('pipeline_ms'), 'v': m.get('v'), 'yaw': m.get('yaw')}
        r.update(flat(m.get('obs'), 'obs')); r.update(flat(m.get('est'), 'est')); r.update(flat(m.get('action'), 'action'))
        rows.append(r)
    if rows:
        keys = sorted({k for r in rows for k in r}, key=lambda k: (k != 't', k))
        with open(f'{out}_telemetry.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
    with open(f'{out}_request.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'forward_m_s', 'yaw_rad_s']); w.writerows([(t, *v) for t, v in zip(data['request'].t, data['request'].v)])
    with open(f'{out}_motors.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'left_m_s', 'right_m_s', 'enabled', 'mode', 'profile']); w.writerows([(t, *v) for t, v in zip(data['motors'].t, data['motors'].v)])
    with open(f'{out}_imu.csv', 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'gyro_z_rad_s', 'acc_x', 'acc_y']); w.writerows([(t, *v) for t, v in zip(data['imu'].t, data['imu'].v)])


def offset_of(m):
    """Offset in px of the 160-wide image from either contract."""
    for key in ('est', 'obs'):
        d = m.get(key) or {}
        if d.get('valid') is False: continue
        if d.get('offset_px') is not None: return float(d['offset_px'])
        if d.get('offset') is not None: return float(d['offset']) * 80
    return float('nan')


def plots(data, out, title):
    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed; skipping plots'); return
    tel = data['telemetry']
    fig, ax = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    if len(tel):
        t = tel.t
        ax[0].plot(t, [offset_of(m) for m in tel.v], lw=.8, label='offset estimate [px of 160]')
        stops = [tt for tt, m in zip(t, tel.v) if (m.get('action') or {}).get('kind', 'follow' if m.get('action') else 'stop') == 'stop']
        if stops: ax[0].scatter(stops, [0] * len(stops), s=5, c='r', label='stop')
        ax[0].axhline(0, c='k', lw=.5); ax[0].legend(loc='upper right'); ax[0].set_ylabel('offset')
        ax[2].plot(t, [m.get('image_age_ms') or np.nan for m in tel.v], lw=.8, label='image age at use [ms]')
        ax[2].plot(t, [m.get('pipeline_ms') or np.nan for m in tel.v], lw=.8, label='pipeline [ms]')
        ax[2].axhline(50, c='r', ls='--', lw=.6); ax[2].legend(loc='upper right'); ax[2].set_ylabel('ms')
    req, mot = data['request'], data['motors']
    if len(req):
        ax[1].plot(req.t, [v[0] for v in req.v], label='requested v [m/s]'); ax[1].plot(req.t, [v[1] for v in req.v], label='requested yaw [rad/s]')
    if len(mot):
        ax[1].plot(mot.t, [v[0] for v in mot.v], lw=.7, label='left tread [m/s]'); ax[1].plot(mot.t, [v[1] for v in mot.v], lw=.7, label='right tread [m/s]')
    ax[1].legend(loc='upper right', fontsize=8)
    imu = data['imu']
    if len(imu):
        ax[3].plot(imu.t, [v[0] for v in imu.v], lw=.6, label='gyro yaw rate [rad/s]')
        if len(req): ax[3].plot(req.t, [v[1] for v in req.v], lw=.8, label='requested yaw [rad/s]')
        ax[3].legend(loc='upper right')
    ax[3].set_xlabel('t [s]'); fig.suptitle(title); fig.tight_layout(); fig.savefig(f'{out}.png', dpi=120)
    print('plot ->', f'{out}.png')


def write_video(data, out, frames_dir, every, fps):
    img = data['image']
    if not len(img):
        print('no camera frames in bag'); return
    first = cv2.imdecode(np.frombuffer(img.v[0], np.uint8), cv2.IMREAD_COLOR)
    h, w = first.shape[:2]
    side = 360
    writer = cv2.VideoWriter(f'{out}.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (w + side, h))
    if frames_dir: Path(frames_dir).mkdir(parents=True, exist_ok=True)
    for i, (t, jpg) in enumerate(zip(img.t, img.v)):
        frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None: continue
        if frames_dir and i % every == 0:
            Path(frames_dir, f'{t:08.3f}.jpg').write_bytes(jpg)
        m = data['telemetry'].at(t) or {}
        req = data['request'].at(t); mot = data['motors'].at(t); saf = data['safety'].at(t) or ''
        act = m.get('action'); kind = (act or {}).get('kind', 'follow' if act else 'stop')
        off = offset_of(m)
        lines = [f't={t:7.2f}s   {kind}   offset={off:+.0f}px' if not np.isnan(off) else f't={t:7.2f}s   {kind}   offset=none',
                 f'request v={req[0]:.2f} yaw={req[1]:+.2f}' if req else 'request: -',
                 f'treads L={mot[0]:+.2f} R={mot[1]:+.2f} {mot[3]} P{mot[4]}' if mot else 'treads: -',
                 f'image age {m.get("image_age_ms") or 0:.0f} ms  pipeline {m.get("pipeline_ms") or 0:.1f} ms',
                 saf[:60]]
        for k, s in enumerate(lines):
            cv2.putText(frame, s, (10, 28 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 0), 4)
            cv2.putText(frame, s, (10, 28 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 1)
        if not np.isnan(off):
            u = int((80 + off) / 160 * w)
            cv2.line(frame, (u, int(h * .6)), (u, int(h * .9)), (0, 0, 255), 3)
            cv2.line(frame, (w // 2, int(h * .6)), (w // 2, int(h * .9)), (255, 255, 255), 1)
        panel = np.full((h, side, 3), 30, np.uint8)
        dbg = data['debug'].at(t)
        if dbg is not None:
            d = cv2.imdecode(np.frombuffer(dbg, np.uint8), cv2.IMREAD_COLOR)
            if d is not None:
                dh = min(h, int(side * d.shape[0] / d.shape[1]))
                panel[:dh] = cv2.resize(d, (side, dh))
        writer.write(np.hstack([frame, panel]))
    writer.release()
    print('video ->', f'{out}.mp4')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag folder (contains metadata.yaml)')
    ap.add_argument('--out', help='output prefix (default: bag folder name)')
    ap.add_argument('--frames', help='also dump JPEG frames here for playground --replay')
    ap.add_argument('--every', type=int, default=3, help='dump every Nth frame (default 3 -> ~10 fps)')
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--no-video', action='store_true')
    a = ap.parse_args()
    out = a.out or Path(a.bag).name
    data = read_bag(a.bag)
    print({k: len(v) for k, v in data.items()})
    tel = data['telemetry']
    if len(tel):
        ages = [m['image_age_ms'] for m in tel.v if m.get('image_age_ms') is not None]
        pipe = [m['pipeline_ms'] for m in tel.v if m.get('pipeline_ms') is not None]
        if ages: print(f'image age at use: median {np.median(ages):.0f} ms, p95 {np.percentile(ages, 95):.0f} ms')
        if pipe: print(f'pipeline: median {np.median(pipe):.1f} ms, max {max(pipe):.1f} ms')
    write_csvs(data, out)
    plots(data, out, Path(a.bag).name)
    if not a.no_video:
        write_video(data, out, a.frames, a.every, a.fps)


if __name__ == '__main__':
    main()
