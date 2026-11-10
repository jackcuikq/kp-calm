"""Check imports and capture plugins without opening camera, UART or I2C."""
import argparse
import importlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--runtime',type=Path,default=Path('/opt/a2r-runtime'))
a=p.parse_args()
sys.path.insert(0,str(a.runtime))
modules=('rclpy','cv_bridge','numpy','cv2','PIL.Image','aiohttp','evdev','serial','serial_asyncio','smbus2',
         'rvr_io','rvr_encoders','jetson_camera','jetson_oled','a2r_state')
for name in modules:importlib.import_module(name)
for program in ('gst-launch-1.0','gst-inspect-1.0','nmcli'):
    if not shutil.which(program):raise SystemExit(f'Missing program: {program}')
for plugin in ('v4l2src','multipartmux','fdsink'):
    subprocess.run(['gst-inspect-1.0',plugin],check=True,stdout=subprocess.DEVNULL)
for font in ('DejaVuSansMono.ttf','DejaVuSans.ttf'):
    if not (Path('/usr/share/fonts/truetype/dejavu')/font).is_file():raise SystemExit(f'Missing font: {font}')
print(json.dumps({'python':sys.executable,'runtime':str(a.runtime),'imports':list(modules),
                  'capture_plugins':'available','devices_opened':False},indent=2))
