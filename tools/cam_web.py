#!/usr/bin/env python3
"""Browser viewer for the ROS camera stream (works while the base launch is running).

Subscribes to <ns>/camera/image_raw/compressed and, if present,
<ns>/navigation/debug/compressed (a perception overlay your node may publish),
and serves both as MJPEG on http://ROBOT_IP:8081/.

Usage, in a prepared Jetson terminal (base launch already running):
    python ~/a2r-student/tools/cam_web.py $ROBOT_NAMESPACE
    python ~/a2r-student/tools/cam_web.py $ROBOT_NAMESPACE --save ~/frames --every 15
Then open http://ROBOT_IP:8081/ on your laptop.
"""
import argparse
import asyncio
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from aiohttp import web

PAGE = """<!doctype html><meta charset=utf-8><title>ROS camera</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px system-ui}
.row{display:flex;flex-wrap:wrap;gap:8px;padding:8px}
figure{margin:0;flex:1 1 480px}img{width:100%;background:#000;display:block}
figcaption{padding:4px 0}</style>
<div class=row>
<figure><img src=/raw.mjpg><figcaption>camera/image_raw/compressed &nbsp; <span id=s></span></figcaption></figure>
<figure><img src=/debug.mjpg><figcaption>navigation/debug/compressed (perception overlay, blank until published)</figcaption></figure>
</div>
<script>setInterval(async()=>{const h=await (await fetch('/health')).json();
document.getElementById('s').textContent=`${h.fps.toFixed(1)} fps, last frame ${h.age.toFixed(2)} s ago`},1000)</script>"""


class Feed:
    def __init__(self):
        self.frame = None
        self.updated = 0.0
        self.count = 0
        self.fps = 0.0
        self._t0 = time.monotonic()
        self.event = asyncio.Event()
        self.loop = None

    def push(self, data):
        self.frame = data
        self.updated = time.monotonic()
        self.count += 1
        if self.updated - self._t0 >= 1:
            self.fps = self.count / (self.updated - self._t0)
            self.count, self._t0 = 0, self.updated
        if self.loop:
            self.loop.call_soon_threadsafe(self._wake)

    def _wake(self):
        self.event.set()
        self.event.clear()


class Viewer(Node):
    def __init__(self, ns, raw, debug, save, every):
        super().__init__('cam_web')
        self.save, self.every, self.n = save, every, 0
        self.create_subscription(CompressedImage, f'/{ns}/camera/image_raw/compressed',
                                 lambda m: self.on_raw(m, raw), qos_profile_sensor_data)
        self.create_subscription(CompressedImage, f'/{ns}/navigation/debug/compressed',
                                 lambda m: debug.push(bytes(m.data)), qos_profile_sensor_data)

    def on_raw(self, msg, feed):
        data = bytes(msg.data)
        feed.push(data)
        if self.save:
            self.n += 1
            if self.n % self.every == 0:
                (self.save / f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec // 1000000:03d}.jpg').write_bytes(data)


async def mjpeg(request, feed):
    resp = web.StreamResponse(headers={'Content-Type': 'multipart/x-mixed-replace; boundary=f',
                                       'Cache-Control': 'no-store'})
    await resp.prepare(request)
    try:
        while True:
            await feed.event.wait()
            if feed.frame is None:
                continue
            await resp.write(b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(feed.frame))
            await resp.write(feed.frame + b'\r\n')
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('namespace')
    ap.add_argument('--port', type=int, default=8081)
    ap.add_argument('--save', type=Path, help='directory to dump raw JPEG frames into')
    ap.add_argument('--every', type=int, default=15, help='save every Nth frame')
    a = ap.parse_args()
    if a.save:
        a.save.mkdir(parents=True, exist_ok=True)

    raw, debug = Feed(), Feed()
    rclpy.init()
    node = Viewer(a.namespace, raw, debug, a.save, a.every)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = web.Application()
    app.router.add_get('/', lambda r: web.Response(text=PAGE, content_type='text/html'))
    app.router.add_get('/raw.mjpg', lambda r: mjpeg(r, raw))
    app.router.add_get('/debug.mjpg', lambda r: mjpeg(r, debug))
    app.router.add_get('/snapshot.jpg', lambda r: web.Response(body=raw.frame or b'', content_type='image/jpeg'))
    app.router.add_get('/health', lambda r: web.json_response(
        {'fps': raw.fps, 'age': time.monotonic() - raw.updated if raw.frame else -1}))

    async def on_startup(app):
        raw.loop = debug.loop = asyncio.get_running_loop()
    app.on_startup.append(on_startup)
    print(f'open http://<ROBOT_IP>:{a.port}/   (Ctrl+C to stop)')
    web.run_app(app, port=a.port, print=None)


if __name__ == '__main__':
    main()
