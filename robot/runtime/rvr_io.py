"""Shared UART/gamepad primitives; contains no staff control/demo commands."""
import asyncio
from pathlib import Path
import sys
from evdev import InputDevice, list_devices, ecodes
sys.path.insert(0, '/opt/a2r-runtime/vendor/sphero-sdk-raspberrypi-python')
from sphero_sdk import SpheroRvrAsync, SerialAsyncDal, RawMotorModesEnum
DEADZONE = .12
OFF = RawMotorModesEnum.off.value
UART_LOCK = Path('/run/lock/a2r-rvr-uart.lock')

class LocalRvr(SpheroRvrAsync):
    async def _check_rvr_fw(self):
        # The SDK constructor runs this via run_until_complete. Avoid its
        # automatic internet check; probe both processors explicitly below.
        pass

def find_gamepad():
    for path in list_devices():
        dev = InputDevice(path)
        if "F710" in dev.name or "Logitech Gamepad" in dev.name:
            return dev
        dev.close()
    raise RuntimeError("F710 not found: plug in receiver and select X mode")

def normalize(value, minimum, maximum):
    center = (minimum + maximum + 1) / 2
    scale = max(center - minimum, maximum - center)
    value = max(-1.0, min(1.0, -(value - center) / scale))
    if abs(value) <= DEADZONE:
        return 0.0
    return (1 if value > 0 else -1) * (abs(value) - DEADZONE) / (1 - DEADZONE)

async def stop(rvr):
    await rvr.raw_motors(OFF, 0, OFF, 0)
    await asyncio.sleep(0.1)

async def probe(rvr):
    await rvr.wake()
    await asyncio.sleep(2)
    for target in (1, 2):
        response = await asyncio.wait_for(
            rvr.get_main_application_version(target=target, timeout=2), 3)
        print(f"RVR processor {target} firmware: {response}", flush=True)
    await rvr.set_custom_control_system_timeout(command_timeout=500)
    await stop(rvr)
    print("UART communication OK; motors commanded OFF.", flush=True)
