#!/usr/bin/env python3
"""128x32 SSD1306 device/Wi-Fi menu. F8 toggles exclusive USB keyboard input."""
import argparse
import asyncio
import contextlib
import fcntl
import json
import logging
import os
from pathlib import Path
import signal
import socket
import time
import textwrap
from a2r_state import read_state as demo_state

from evdev import InputDevice, ecodes, list_devices
from PIL import Image, ImageDraw, ImageFont
from smbus2 import SMBus

LOG = logging.getLogger("oled")
STATE = Path.home() / ".local/state/rvr-oled"


def split_terse(line):
    fields, field, escape = [], "", False
    for char in line:
        if escape:
            field += char
            escape = False
        elif char == "\\":
            escape = True
        elif char == ":":
            fields.append(field)
            field = ""
        else:
            field += char
    fields.append(field)
    return fields


async def nmcli(*args, privileged=False, password=None):
    command = (["sudo", "-n"] if privileged else []) + ["nmcli", "--colors", "no", "--wait", "25"]
    if password is not None:
        command.append("--ask")
    command.extend(args)
    proc = await asyncio.create_subprocess_exec(
        *command, stdin=asyncio.subprocess.PIPE if password is not None else asyncio.subprocess.DEVNULL,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
        env={**os.environ, "LC_ALL": "C"})
    try:
        out, err = await asyncio.wait_for(proc.communicate(
            (password + "\n").encode() if password is not None else None), 35)
    except BaseException:
        with contextlib.suppress(ProcessLookupError):
            proc.kill()
        await proc.wait()
        raise
    if proc.returncode:
        # Never include nmcli's captured output in logs: --ask can echo input.
        raise RuntimeError(f"Wi-Fi command failed (code {proc.returncode})")
    return out.decode(errors="replace").strip()


class Display:
    def __init__(self, bus=7, address=0x3c):
        self.bus = SMBus(bus)
        self.address = address
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 8)
        self.previous = None
        # SSD1306 128x32, internal charge pump, horizontal addressing.
        self.command([0xae, 0xd5, 0x80, 0xa8, 0x1f, 0xd3, 0, 0x40,
                      0x8d, 0x14, 0x20, 0, 0xa1, 0xc8, 0xda, 0x02,
                      0x81, 0x7f, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0xaf])

    def command(self, data):
        for start in range(0, len(data), 32):
            self.bus.write_i2c_block_data(self.address, 0, data[start:start + 32])

    def show(self, lines):
        if lines == self.previous:
            return
        image = Image.new("1", (128, 32))
        draw = ImageDraw.Draw(image)
        for row, text in enumerate(lines[:4]):
            draw.text((0, row * 8 - 2), text, font=self.font, fill=1)
        data = [sum((1 << bit) for bit in range(8) if image.getpixel((x, page * 8 + bit)))
                for page in range(4) for x in range(128)]
        self.command([0x21, 0, 127, 0x22, 0, 3])
        for start in range(0, len(data), 32):
            self.bus.write_i2c_block_data(self.address, 0x40, data[start:start + 32])
        self.previous = list(lines)
        STATE.mkdir(parents=True, exist_ok=True)
        image.save(STATE / "screen.png")
        # Contains rendered text only; password text is always masked.
        temp = STATE / "screen.tmp"
        temp.write_text(json.dumps({"lines": lines, "updated": time.time()}))
        temp.replace(STATE / "screen.json")

    def close(self):
        self.command([0xae])
        self.bus.close()


def character(code, shift=False, caps=False):
    name = ecodes.KEY.get(code, "")
    if isinstance(name, tuple):
        name = name[0]
    letter = name.removeprefix("KEY_")
    if len(letter) == 1 and letter.isalpha():
        return letter.upper() if shift != caps else letter.lower()
    pairs = {"KEY_1": "1!", "KEY_2": "2@", "KEY_3": "3#", "KEY_4": "4$",
             "KEY_5": "5%", "KEY_6": "6^", "KEY_7": "7&", "KEY_8": "8*",
             "KEY_9": "9(", "KEY_0": "0)", "KEY_MINUS": "-_", "KEY_EQUAL": "=+",
             "KEY_LEFTBRACE": "[{", "KEY_RIGHTBRACE": "]}", "KEY_BACKSLASH": "\\|",
             "KEY_SEMICOLON": ";:", "KEY_APOSTROPHE": "'\"", "KEY_GRAVE": "`~",
             "KEY_COMMA": ",<", "KEY_DOT": ".>", "KEY_SLASH": "/?", "KEY_SPACE": "  "}
    return pairs.get(name, "")[int(shift):int(shift) + 1]


class Menu:
    def __init__(self, display, name="A2R Jetson"):
        self.display = display
        self.name = name
        self.active = False
        self.page = "home"
        self.selection = 0
        self.networks = []
        self.selected = None
        self.secret = ""
        self.notice = ""
        self.wifi = "disconnected"
        self.ip = "No network IP"
        self.interface = None
        self.device = None
        self.operation = None
        self.last_input = time.monotonic()
        self.last_toggle = 0.0
        self.caps = False

    def hide(self):
        self.active = False
        self.secret = ""
        self.page = "home"
        if self.device:
            with contextlib.suppress(OSError):
                self.device.ungrab()

    def toggle(self, device=None):
        if device is not None and self.active and self.device is not device:
            return
        if self.active:
            self.hide()
        else:
            if device is not None:
                self.device = device
            if not self.device:
                return
            self.device.grab()
            self.caps = ecodes.LED_CAPSL in self.device.leds()
            self.active, self.page, self.selection = True, "home", 0
            self.last_input = time.monotonic()

    def start(self, coroutine):
        if self.operation and not self.operation.done():
            coroutine.close()
            return
        async def guarded():
            try:
                await coroutine
            except Exception as exc:
                LOG.warning("Menu operation failed: %s", type(exc).__name__)
                self.secret = ""
                self.notice = str(exc)
                self.page = "message"
        self.operation = asyncio.create_task(guarded())

    async def refresh(self):
        rows = [split_terse(line) for line in (await nmcli("-t", "-f", "DEVICE,TYPE,STATE", "device")).splitlines()]
        self.interface = next((row[0] for row in rows if len(row) >= 3 and row[1] == "wifi"), None)
        self.wifi = "No Wi-Fi adapter" if not self.interface else "disconnected"
        if self.interface:
            networks = await nmcli("-t", "-f", "IN-USE,SSID", "device", "wifi", "list",
                                   "ifname", self.interface, "--rescan", "no")
            active = [split_terse(line) for line in networks.splitlines()]
            self.wifi = next((row[1] for row in active if len(row) == 2 and row[0] == "*" and row[1]),
                             "disconnected")
        addresses = await nmcli("-g", "IP4.ADDRESS", "device", "show", self.interface) if self.interface else ""
        self.ip = addresses.splitlines()[0].split("/")[0] if addresses else "No Wi-Fi IP"

    async def scan(self):
        self.page = "busy"
        self.notice = "Scanning Wi-Fi..."
        await self.refresh()
        if not self.interface:
            raise RuntimeError("No Wi-Fi adapter")
        await nmcli("radio", "wifi", "on", privileged=True)
        text = await nmcli("-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE,BSSID", "device", "wifi", "list",
                           "ifname", self.interface, "--rescan", "yes", privileged=True)
        networks = {}
        for line in text.splitlines():
            row = split_terse(line)
            if len(row) != 5 or not row[0]:
                continue
            ssid, strength, security, current, bssid = row
            item = dict(ssid=ssid, signal=int(strength), security=security, current=current == "*", bssid=bssid)
            if ssid not in networks or item["signal"] > networks[ssid]["signal"]:
                networks[ssid] = item
        self.networks = sorted(networks.values(), key=lambda n: (-n["current"], -n["signal"], n["ssid"]))
        self.page, self.selection = "networks", 0
        if not self.networks:
            self.page, self.notice = "message", "No networks found"

    async def choose(self):
        self.page, self.notice = "busy", "Checking network..."
        # Saved enterprise profiles are supported without weakening certificates.
        uuids = await nmcli("-t", "-f", "UUID,TYPE", "connection", "show")
        for line in uuids.splitlines():
            row = split_terse(line)
            if len(row) == 2 and row[1] == "802-11-wireless":
                ssid = await nmcli("-g", "802-11-wireless.ssid", "connection", "show", "uuid", row[0])
                if ssid == self.selected["ssid"]:
                    self.selected["uuid"] = row[0]
                    self.page = "confirm"
                    return
        security = self.selected["security"]
        if "802.1X" in security:
            self.page, self.notice = "message", "Enterprise Wi-Fi: configure profile first"
        elif security and security != "--":
            if "WEP" in security:
                self.page, self.notice = "message", "WEP not supported"
            else:
                self.secret, self.page = "", "password"
        else:
            self.page = "confirm"

    async def connect(self, secret):
        self.page, self.notice = "busy", "Connecting..."
        if self.selected.get("uuid"):
            await nmcli("connection", "up", "uuid", self.selected["uuid"], "ifname", self.interface, privileged=True)
        else:
            await nmcli("device", "wifi", "connect", self.selected["bssid"], "ifname", self.interface,
                        privileged=True, password=secret or None)
        await self.refresh()
        self.page, self.notice = "message", "Wi-Fi connected"

    def key(self, code, shift=False, caps=False):
        self.last_input = time.monotonic()
        if self.page == "busy":
            return
        if self.page == "password":
            if code == ecodes.KEY_BACKSPACE:
                self.secret = self.secret[:-1]
            elif code in (ecodes.KEY_ENTER, ecodes.KEY_KPENTER):
                if not (8 <= len(self.secret) <= 63 or len(self.secret) == 64 and all(c in "0123456789abcdefABCDEF" for c in self.secret)):
                    self.notice = "Use 8-63 characters"
                else:
                    secret, self.secret = self.secret, ""
                    self.start(self.connect(secret))
            else:
                char = character(code, shift, caps)
                if char and len(self.secret) < 64:
                    self.secret += char
                    self.notice = ""
            return
        if code in (ecodes.KEY_UP, ecodes.KEY_DOWN):
            count = len(self.networks) if self.page == "networks" else 3
            if count:
                self.selection = (self.selection + (1 if code == ecodes.KEY_DOWN else -1)) % count
        elif code in (ecodes.KEY_ENTER, ecodes.KEY_KPENTER):
            if self.page == "home":
                if self.selection == 0:
                    self.start(self.scan())
                elif self.selection == 1:
                    self.page = "info"
                else:
                    self.hide()
            elif self.page == "networks":
                self.selected = self.networks[self.selection].copy()
                self.start(self.choose())
            elif self.page == "confirm":
                self.start(self.connect(""))
            else:
                self.page, self.selection = "home", 0

    def back(self):
        self.secret = ""
        if self.page in ("home", "busy"):
            self.hide()
        else:
            self.page, self.selection = "home", 0

    def lines(self):
        demo = demo_state()
        if demo["active"]:
            return textwrap.wrap(demo["message"], width=25)[:4] or ["HACKED"]
        if not self.active:
            return [self.name, "WiFi: " + self.wifi, self.ip, "F8: menu"]
        if self.page == "home":
            return [self.name] + [("> " if self.selection == n else "  ") + item
                                     for n, item in enumerate(("Connect Wi-Fi", "Device information", "Close menu"))]
        if self.page == "networks":
            start = self.selection // 3 * 3
            return [f"Wi-Fi {self.selection + 1}/{len(self.networks)}"] + [
                (">" if n == self.selection else " ") + f"{item['signal']:3}% " + item["ssid"]
                for n, item in enumerate(self.networks[start:start + 3], start)]
        if self.page == "password":
            return [self.selected["ssid"], "Password (US keys):", ("*" * len(self.secret))[-24:] + "_",
                    self.notice or "Enter:join Esc:back"]
        if self.page == "confirm":
            return ["Connect Wi-Fi?", self.selected["ssid"], "Enter: connect", "Esc: back"]
        if self.page == "info":
            return [self.name, socket.gethostname(), "WiFi: " + self.wifi, self.ip]
        text = self.notice
        return [text[i:i + 26] for i in (0, 26, 52)] + (["Enter: back"] if self.page == "message" else ["Esc: hide menu"])

    async def render_loop(self):
        while True:
            if self.active and self.page != "busy" and time.monotonic() - self.last_input > 90:
                self.hide()
            self.display.show(self.lines())
            await asyncio.sleep(0.1)

    async def status_loop(self):
        while True:
            try:
                await self.refresh()
            except Exception:
                LOG.warning("Status refresh unavailable")
            await asyncio.sleep(5)

    async def watch_keyboard(self, device):
        LOG.info("Keyboard available: %s (%s)", device.name, device.path)
        try:
            async for event in device.async_read_loop():
                if event.type != ecodes.EV_KEY:
                    continue
                # Listen to every attached keyboard; F8 chooses the active one.
                # Some keyboards expose duplicate interfaces, so debounce F8.
                if event.code == ecodes.KEY_F8 and event.value == 0:
                    if time.monotonic() - self.last_toggle > 0.3:
                        if not self.active or self.device is device:
                            self.toggle(device)
                            self.last_toggle = time.monotonic()
                elif self.active and self.device is device:
                    if event.code == ecodes.KEY_ESC and event.value == 0:
                        self.back()
                    elif event.code == ecodes.KEY_CAPSLOCK and event.value == 1:
                        self.caps = not self.caps
                    elif event.value in (1, 2):
                        keys = device.active_keys()
                        self.key(event.code, bool({ecodes.KEY_LEFTSHIFT, ecodes.KEY_RIGHTSHIFT} & set(keys)), self.caps)
        except OSError:
            LOG.info("Keyboard disconnected: %s", device.path)
        finally:
            if self.device is device:
                self.hide()
                self.device = None
            device.close()

    async def keyboard_loop(self):
        watchers = {}
        try:
            while True:
                for path, task in list(watchers.items()):
                    if task.done():
                        task.result()
                        del watchers[path]
                for path in list_devices():
                    if path in watchers:
                        continue
                    try:
                        device = InputDevice(path)
                        keys = device.capabilities().get(ecodes.EV_KEY, [])
                        if ecodes.KEY_A in keys and ecodes.KEY_F8 in keys:
                            watchers[path] = asyncio.create_task(self.watch_keyboard(device))
                        else:
                            device.close()
                    except OSError:
                        continue
                await asyncio.sleep(2)
        finally:
            for task in watchers.values():
                task.cancel()
            await asyncio.gather(*watchers.values(), return_exceptions=True)


async def run(args):
    display = Display(args.bus, args.address)
    menu = Menu(display, args.name)
    tasks = [asyncio.create_task(coro) for coro in (menu.render_loop(), menu.status_loop(), menu.keyboard_loop())]
    try:
        await asyncio.gather(*tasks)
    finally:
        for task in tasks:
            task.cancel()
        if menu.operation:
            menu.operation.cancel()
            tasks.append(menu.operation)
        await asyncio.gather(*tasks, return_exceptions=True)
        menu.hide()
        display.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=7)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x3c)
    parser.add_argument("--name", default="A2R Jetson")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    STATE.mkdir(parents=True, exist_ok=True)
    with open(STATE / "menu.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            asyncio.run(run(args))
        except KeyboardInterrupt:
            pass
