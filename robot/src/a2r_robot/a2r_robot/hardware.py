"""Exclusive hardware owner. ROS callbacks and UART operations share one asyncio loop."""
import asyncio
import contextlib
import fcntl
import math
from pathlib import Path
import signal
import sys
import time
import uuid

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Image, CompressedImage, Joy
from std_srvs.srv import Trigger
from a2r_robot_interfaces.msg import EncoderState, HardwareStatus, JoystickState, TankCommand
from .guard import Deadman, Pad, SpeedSelector, validate_command, TIMEOUT, SPEEDS
from .safety import CONTROL_QOS, stamp_age

# Existing proven driver and camera capture helpers, without starting their web viewers.
sys.path.insert(0, "/opt/a2r-runtime")
from rvr_io import LocalRvr, probe, stop, find_gamepad, normalize, OFF
from sphero_sdk import SerialAsyncDal
from evdev import ecodes
from rvr_encoders import EncoderDistance
from a2r_state import read_state as demo_state
from jetson_camera import Camera, CAMERA

BUTTONS = (ecodes.BTN_SOUTH, ecodes.BTN_EAST, ecodes.BTN_NORTH, ecodes.BTN_Y,
           ecodes.BTN_TL, ecodes.BTN_TR, ecodes.BTN_SELECT, ecodes.BTN_START,
           ecodes.BTN_MODE, ecodes.BTN_THUMBL, ecodes.BTN_THUMBR)
AXES = (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY)
SERVICES = ('IMU', 'Quaternion', 'Accelerometer', 'Gyroscope', 'Encoders')


class Hardware(Node):
    def __init__(self):
        super().__init__('rvr_hardware')
        self.session = uuid.uuid4().hex
        self.uart = self.declare_parameter('uart', '/dev/ttyTHS1').value
        self.motion_enabled = self.declare_parameter('motion_enabled', False).value
        camera_device = self.declare_parameter('camera_device', CAMERA).value
        self.raw_images = self.declare_parameter('publish_raw_images', True).value
        self.frame_prefix = self.get_namespace().strip('/') or 'jetson1'
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.encoder_pub = self.create_publisher(EncoderState, 'encoders', qos_profile_sensor_data)
        self.joy_pub = self.create_publisher(Joy, 'joy', CONTROL_QOS)
        self.pad_pub = self.create_publisher(JoystickState, 'joystick/state', CONTROL_QOS)
        self.status_pub = self.create_publisher(HardwareStatus, 'hardware/status', CONTROL_QOS)
        self.jpeg_pub = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.create_subscription(TankCommand, 'motors/command', self.command_cb, CONTROL_QOS)
        self.create_service(Trigger, 'motors/disable', self.disable_cb)
        self.rvr = None
        self.pad = None
        self.pad_at = self.command_at = -math.inf
        self.command = None
        self.sequence = self.y_presses = 0
        self.device = None
        self.next_connect = 0
        self.next_graph_check = 0
        self.publisher_ok = False
        self.local_deadman = Deadman()
        self.speed_selector = SpeedSelector()
        self.last = {}
        self.encoder = EncoderDistance()
        self.uart_ok = False
        self.link_ok = False
        self.left = self.right = 0.
        self.reason = 'starting; motors disabled' if not self.motion_enabled else 'waiting for safety'
        self.camera = Camera(camera_device)
        self.camera_task = None
        self.imu_frame = f'{self.frame_prefix}/imu_link'
        self.get_logger().info(f'session={self.session}; motion_enabled={self.motion_enabled}; {self.encoder.notice}')

    def disable_cb(self, _request, response):
        self.motion_enabled = False
        self.command = None
        response.success, response.message = True, 'Motion disabled until hardware node restart.'
        return response

    def command_cb(self, msg):
        # Graph identity is a wiring/misconfiguration check, not DDS authentication.
        if time.monotonic()-self.command_at > TIMEOUT:
            self.local_deadman.disarm('safety command gap')
        if (not -.05 <= stamp_age(msg.header.stamp, self) <= TIMEOUT or
                msg.hardware_session != self.session or
                msg.control_mode not in ('stopped', 'manual', 'autonomous') or
                msg.speed_profile not in range(len(SPEEDS)) or
                any(not math.isfinite(v) or abs(v) > SPEEDS[-1]
                    for v in (msg.left_m_s, msg.right_m_s))):
            self.command = None
            self.local_deadman.disarm('invalid safety command')
            return
        self.command, self.command_at = msg, time.monotonic()

    def fresh(self, key):
        return time.monotonic()-self.last.get(key, -math.inf) < 1.

    def healthy(self):
        return (self.uart_ok and self.link_ok and
                all(self.fresh(k) for k in ('Quaternion','Accelerometer','Gyroscope','Encoders'))
                and not demo_state()['locked'])

    def sample_pad(self):
        now = time.monotonic()
        self.sequence += 1
        msg = JoystickState()
        msg.hardware_session, msg.sequence = self.session, self.sequence
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = f'{self.frame_prefix}/gamepad'
        try:
            if self.device is None:
                if now < self.next_connect:
                    raise OSError('waiting for joystick')
                self.device = find_gamepad()
                self.get_logger().info(f'Joystick: {self.device.name} {self.device.path}')
            dropped = False
            with contextlib.suppress(BlockingIOError):
                for event in self.device.read():
                    if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_DROPPED:
                        dropped = True
                    elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_Y and event.value == 1:
                        self.y_presses += 1
            keys = self.device.active_keys()
            joy.axes = [float(normalize(v.value, v.min, v.max))
                        for v in (self.device.absinfo(code) for code in AXES)]
            joy.buttons = [int(code in keys) for code in BUTTONS]
            msg.connected, msg.events_lost = True, dropped
        except (OSError, RuntimeError, AttributeError) as exc:
            if self.device:
                self.get_logger().warning(f'Joystick disconnected: {exc}')
                self.device.close()
                self.device = None
            if now >= self.next_connect:
                self.next_connect = now+1.
            joy.axes, joy.buttons = [0.]*4, [0]*len(BUTTONS)
            msg.connected, msg.events_lost = False, True
        msg.joy, msg.y_presses = joy, self.y_presses
        self.pad = Pad(self.session, self.sequence, joy.axes[1], joy.axes[3],
                       bool(joy.buttons[5]), bool(joy.buttons[0]), bool(joy.buttons[3]),
                       self.y_presses, msg.connected and not msg.events_lost,
                       bool(joy.buttons[4]))
        self.pad.speed_profile = self.speed_selector.update(self.pad, now, self.healthy())
        msg.speed_profile = self.pad.speed_profile
        self.pad_at = now
        self.joy_pub.publish(joy)
        self.pad_pub.publish(msg)

    async def sensors(self, data):
        now = time.monotonic()
        valid = {k:v for k,v in data.items() if v.get('is_valid',True) and
                 all(math.isfinite(x) for field,x in v.items() if field != 'is_valid')}
        if 'Quaternion' in valid and sum(valid['Quaternion'][k]**2 for k in ('W','X','Y','Z')) < .01:
            del valid['Quaternion']
        for key in valid:
            self.last[key] = now
        if all(k in valid for k in ('Quaternion','Accelerometer','Gyroscope')):
            q = valid['Quaternion']
            w,x,y,z = (q[k] for k in ('W','X','Y','Z'))
            norm = math.sqrt(w*w+x*x+y*y+z*z)
            if norm > .1:
                imu = Imu()
                imu.header.stamp = self.get_clock().now().to_msg()
                imu.header.frame_id = self.imu_frame
                # Physically validated Viser attitude mapping, WXYZ -> W,Y,-X,-Z.
                imu.orientation.w, imu.orientation.x = w/norm, y/norm
                imu.orientation.y, imu.orientation.z = -x/norm, -z/norm
                # Body axes forward/left/up; SI units, gravity retained.
                a, g = valid['Accelerometer'], valid['Gyroscope']
                imu.linear_acceleration.x = a['Y']*9.80665
                imu.linear_acceleration.y = -a['X']*9.80665
                imu.linear_acceleration.z = a['Z']*9.80665
                imu.angular_velocity.x = math.radians(g['Y'])
                imu.angular_velocity.y = math.radians(-g['X'])
                imu.angular_velocity.z = math.radians(g['Z'])
                # Zero covariance arrays mean UNKNOWN, not measured zero noise.
                self.imu_pub.publish(imu)
        if 'Encoders' in valid:
            e = valid['Encoders']
            self.encoder.update(e, now)
            msg = EncoderState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'{self.frame_prefix}/base_link'
            msg.hardware_session = self.session
            msg.left_ticks, msg.right_ticks = int(e['LeftTicks']), int(e['RightTicks'])
            msg.left_relative_ticks, msg.right_relative_ticks = self.encoder.ticks
            r = self.encoder.reading()
            msg.calibrated = r is not None
            msg.calibration_method = self.encoder.method
            for field in ('left_distance_m','right_distance_m','left_delta_m','right_delta_m',
                          'left_speed_m_s','right_speed_m_s','sample_dt_s'):
                value = r.get(field) if r else None
                setattr(msg, field, float(value) if value is not None else math.nan)
            msg.left_metres_per_tick, msg.right_metres_per_tick = self.encoder.scales or [math.nan,math.nan]
            self.encoder_pub.publish(msg)

    def publish_status(self):
        s = HardwareStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.hardware_session = self.session
        s.uart_ok = self.uart_ok and self.link_ok
        s.imu_fresh = all(self.fresh(k) for k in ('Quaternion','Accelerometer','Gyroscope'))
        s.encoders_fresh = self.fresh('Encoders')
        s.joystick_connected = bool(self.pad and self.pad.valid)
        s.camera_fresh = self.camera.healthy()
        s.demo_locked = bool(demo_state()['locked'])
        s.drive_enabled = self.motion_enabled
        s.applied_left_m_s, s.applied_right_m_s = self.left, self.right
        s.reason = self.reason
        self.status_pub.publish(s)

    def publish_camera_frame(self, frame, stamp):
        # Encoding, large message copies and DDS writes must not block the
        # asyncio loop that refreshes motor commands and receives the UART.
        import cv2
        import numpy as np
        msg = CompressedImage()
        msg.header.stamp, msg.header.frame_id = stamp, f'{self.frame_prefix}/camera_optical_frame'
        msg.format = 'jpeg'
        msg.data = frame
        self.jpeg_pub.publish(msg)
        if self.raw_images and self.image_pub.get_subscription_count():
            image = cv2.imdecode(np.frombuffer(frame,dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is not None:
                raw = Image()
                raw.header = msg.header
                raw.height,raw.width = image.shape[:2]
                raw.encoding,raw.is_bigendian,raw.step = 'bgr8',0,raw.width*3
                raw.data = image.tobytes()
                self.image_pub.publish(raw)

    async def camera_loop(self):
        capture = asyncio.create_task(self.camera.capture())
        sequence = -1
        try:
            while True:
                async with self.camera.condition:
                    await self.camera.condition.wait_for(lambda: sequence != self.camera.sequence)
                    frame, sequence = self.camera.frame, self.camera.sequence
                    stamp = self.get_clock().now().to_msg()
                if frame is None:
                    continue
                await asyncio.to_thread(self.publish_camera_frame,frame,stamp)
        finally:
            capture.cancel()
            await asyncio.gather(capture, return_exceptions=True)

    async def run(self):
        health_task = None
        try:
            await probe(self.rvr)
            self.uart_ok = self.link_ok = True
            async def ignore(_):
                pass
            for service in SERVICES:
                await self.rvr.sensor_control.add_sensor_data_handler(
                    service, self.sensors if service in ('IMU','Encoders') else ignore)
            await self.rvr.sensor_control.start(interval=50)
            self.camera_task = asyncio.create_task(self.camera_loop())
            next_health = 0.
            while rclpy.ok():
                began = time.monotonic()
                # Bound executor work; do not starve the motor watchdog.
                for _ in range(6):
                    rclpy.spin_once(self, timeout_sec=0)
                self.sample_pad()
                if self.camera_task.done():
                    self.camera_task.result()
                    raise RuntimeError('camera worker ended unexpectedly')
                if began >= self.next_graph_check:
                    pubs = self.get_publishers_info_by_topic(self.get_namespace().rstrip('/')+'/motors/command')
                    self.publisher_ok = (len(pubs)==1 and pubs[0].node_name=='rvr_safety' and
                                         pubs[0].node_namespace==self.get_namespace())
                    self.next_graph_check = began+.5
                if health_task and health_task.done():
                    try:
                        health_task.result()
                        self.link_ok = True
                    except Exception as exc:
                        self.link_ok = False
                        self.get_logger().error(f'UART health failed: {exc}')
                    health_task = None
                if began >= next_health and health_task is None:
                    async def health():
                        await asyncio.wait_for(self.rvr.get_main_application_version(target=2,timeout=.35),.4)
                    health_task = asyncio.create_task(health())
                    next_health = began+1.
                c = self.command
                gate_now = time.monotonic()
                self.local_deadman.update(self.pad, gate_now, self.healthy() and self.publisher_ok,
                                          self.get_clock().now().nanoseconds/1e9)
                if gate_now-self.command_at > TIMEOUT:
                    self.local_deadman.disarm('safety command expired')
                local_enabled = self.local_deadman.command(gate_now, self.healthy() and self.publisher_ok)[2]
                allowed = bool(c and self.publisher_ok and validate_command(
                    self.session,c.hardware_session,c.joystick_sequence,self.sequence,
                    gate_now-self.pad_at,gate_now-self.command_at,stamp_age(c.header.stamp,self),
                    c.left_m_s,c.right_m_s,c.enabled and local_enabled,self.pad,self.healthy(),self.motion_enabled,
                    command_mode=c.control_mode, selected_mode=self.local_deadman.mode,
                    source_stamp=c.header.stamp.sec+c.header.stamp.nanosec/1e9,
                    mode_entered=self.local_deadman.entered_ros,
                    command_profile=c.speed_profile))
                self.left, self.right = (c.left_m_s,c.right_m_s) if allowed else (0.,0.)
                self.reason = ('safety command accepted' if allowed else
                               'motion disabled' if not self.motion_enabled else
                               'waiting for fresh safe command / selected bumper / healthy hardware')
                if self.left or self.right:
                    await self.rvr.drive_tank_si_units(self.left,self.right)
                else:
                    await self.rvr.raw_motors(OFF,0,OFF,0)
                self.publish_status()
                await asyncio.sleep(max(0.,.05-(time.monotonic()-began)))
        finally:
            self.uart_ok = False
            self.left = self.right = 0.
            if health_task:
                health_task.cancel()
                await asyncio.gather(health_task,return_exceptions=True)
            if self.camera_task:
                self.camera_task.cancel()
                await asyncio.gather(self.camera_task,return_exceptions=True)
            if self.device:
                self.device.close()
            with contextlib.suppress(Exception):
                await asyncio.wait_for(stop(self.rvr),1.)
            with contextlib.suppress(Exception):
                await asyncio.wait_for(self.rvr.sensor_control.clear(),1.)
            await self.rvr.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Hardware()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    lock = open(Path('/run/lock/a2r-rvr-uart.lock'),'a')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        # The SDK constructor uses run_until_complete internally: construct
        # before starting the asyncio loop, after acquiring exclusive UART ownership.
        node.rvr = LocalRvr(dal=SerialAsyncDal(loop=loop,port_id=node.uart))
    except BaseException:
        lock.close()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()
        raise
    task = loop.create_task(node.run())
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        loop.add_signal_handler(sig,task.cancel)
    try:
        loop.run_until_complete(task)
    except (asyncio.CancelledError, ExternalShutdownException):
        pass
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.run_until_complete(loop.shutdown_default_executor())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        loop.close()
        lock.close()
