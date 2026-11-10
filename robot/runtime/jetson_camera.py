#!/usr/bin/env python3
"""Serve the UC684's native MJPEG stream to browsers without re-encoding."""
import argparse
import asyncio
import contextlib
import logging
from pathlib import Path
import time
import io
import textwrap
from PIL import Image, ImageDraw, ImageFont
from a2r_state import read_state as demo_state

from aiohttp import web

LOG = logging.getLogger("camera")


def demo_overlay(frame, state):
    if not state["active"]:
        return frame
    image = Image.open(io.BytesIO(frame)).convert("RGB")
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 26)
    lines = textwrap.wrap(state["message"], width=65)[:3] or ["HACKED"]
    draw.rectangle((0, 0, image.width, 20 + 34 * len(lines)), fill=(155, 15, 25))
    draw.multiline_text((16, 10), "\n".join(lines), font=font, fill="white", spacing=8)
    out = io.BytesIO()
    image.save(out, format="JPEG", quality=85)
    return out.getvalue()
CAMERA = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_1080P_Low_Light_UC684-video-index0"
PAGE = """<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Jetson camera</title>
<style>
body{margin:0;background:#10151d;color:#e6edf3;font:16px system-ui}
header{display:flex;align-items:center;justify-content:space-between;padding:16px 24px;gap:16px}
h1{font-size:20px;margin:0}#status{color:#9caebf;font-size:14px}
main{max-width:1280px;margin:auto}img{width:100%;display:block;background:#080b10;aspect-ratio:16/9;object-fit:contain}
footer{padding:16px 24px;color:#9caebf;font-size:14px}a{color:#8acaff}
</style>
<header><h1>Jetson camera</h1><span id="status">Connecting…</span></header>
<main><img id="feed" src="/stream.mjpg" alt="Live Arducam UC684 video"></main>
<footer>Arducam UC684 · 1280 × 720 · 30 fps · <a href="/snapshot.jpg" target="_blank">Open snapshot</a></footer>
<script>
const feed=document.getElementById('feed'), status=document.getElementById('status');
feed.onerror=()=>setTimeout(()=>{feed.src='/stream.mjpg?t='+Date.now()},1500);
async function check(){
 try{const r=await fetch('/health',{cache:'no-store'});const h=await r.json();
 status.textContent=r.ok?'Live · '+h.fps+' fps':'Camera reconnecting…';
 status.style.color=r.ok?'#6ee7ad':'#f5c168';
 }catch{status.textContent='Feed unavailable';status.style.color='#f5c168'}
}check();setInterval(check,2000);
</script></html>"""


async def read_frame(reader):
    await reader.readuntil(b"--frame\r\n")
    header = await reader.readuntil(b"\r\n\r\n")
    headers = dict(line.split(b":", 1) for line in header.strip().split(b"\r\n"))
    size = int(headers[b"Content-Length"].strip())
    if not 0 < size < 8 * 1024 * 1024:
        raise ValueError(f"Invalid JPEG size: {size}")
    return await reader.readexactly(size)


class Camera:
    def __init__(self, device):
        self.device = device
        self.condition = asyncio.Condition()
        self.frame = None
        self.sequence = 0
        self.updated = 0.0
        self.fps = 0.0
        self.clients = 0

    async def capture(self):
        while True:
            process = None
            try:
                process = await asyncio.create_subprocess_exec(
                    "gst-launch-1.0", "-q", "v4l2src", f"device={self.device}",
                    "!", "image/jpeg,width=1280,height=720,framerate=30/1",
                    "!", "multipartmux", "boundary=frame", "!", "fdsink", "fd=1",
                    stdout=asyncio.subprocess.PIPE,
                )
                started = time.monotonic()
                count = 0
                while True:
                    frame = await asyncio.wait_for(read_frame(process.stdout), 5)
                    state = demo_state()
                    if state["active"]:
                        frame = await asyncio.to_thread(demo_overlay, frame, state)
                    async with self.condition:
                        self.frame = frame
                        self.sequence += 1
                        self.updated = time.monotonic()
                        count += 1
                        elapsed = self.updated - started
                        if elapsed >= 1:
                            self.fps = round(count / elapsed, 1)
                            started, count = self.updated, 0
                        self.condition.notify_all()
            except (OSError, ValueError, asyncio.IncompleteReadError, asyncio.LimitOverrunError, TimeoutError):
                LOG.exception("Camera capture interrupted; retrying")
            finally:
                if process is not None:
                    if process.returncode is None:
                        with contextlib.suppress(ProcessLookupError):
                            process.terminate()
                    try:
                        await asyncio.wait_for(process.wait(), 3)
                    except TimeoutError:
                        with contextlib.suppress(ProcessLookupError):
                            process.kill()
                        await process.wait()
            await asyncio.sleep(1)

    def healthy(self):
        return self.frame is not None and time.monotonic() - self.updated < 3

    async def stream(self, request):
        if not self.healthy():
            raise web.HTTPServiceUnavailable(text="Camera is reconnecting; try again shortly")
        response = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "X-Accel-Buffering": "no",
        })
        await response.prepare(request)
        self.clients += 1
        sequence = -1
        try:
            while True:
                async with self.condition:
                    await asyncio.wait_for(self.condition.wait_for(lambda: self.sequence != sequence), 5)
                    frame, sequence = self.frame, self.sequence
                part = (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                # Keep only the latest frame: slow clients cannot build a queue.
                await asyncio.wait_for(response.write(part), 2)
        except (ConnectionError, TimeoutError):
            pass
        finally:
            self.clients -= 1
        return response

    async def snapshot(self, request):
        if not self.healthy():
            raise web.HTTPServiceUnavailable(text="Camera unavailable")
        return web.Response(body=self.frame, content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def health(self, request):
        return web.json_response({
            "ok": self.healthy(), "camera": "Arducam UC684", "width": 1280, "height": 720,
            "fps": self.fps, "frames": self.sequence, "viewers": self.clients,
            "frame_age_seconds": round(time.monotonic() - self.updated, 3) if self.updated else None,
        }, status=200 if self.healthy() else 503, headers={"Cache-Control": "no-store"})


def make_app(device):
    camera = Camera(device)
    app = web.Application()

    async def lifecycle(app):
        task = asyncio.create_task(camera.capture())
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    async def index(request):
        return web.Response(text=PAGE, content_type="text/html")

    app.cleanup_ctx.append(lifecycle)
    app.router.add_get("/", index)
    app.router.add_get("/stream.mjpg", camera.stream)
    app.router.add_get("/snapshot.jpg", camera.snapshot)
    app.router.add_get("/health", camera.health)
    return app


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=CAMERA)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    web.run_app(make_app(args.device), host=args.host, port=args.port, access_log=None)
