"""Hardware-free checks. These do not certify ROS transport or physical stops."""
import importlib.util
import math
from pathlib import Path
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robot/src/a2r_robot'))
from a2r_robot.guard import Deadman, MotionGuard, Pad, SpeedSelector, SPEEDS, validate_command, TREAD_LIMIT


def pad(guard, t, **kwargs):
    sequence = guard.pad.sequence+1 if guard.pad else 1
    guard.update(Pad('session', sequence, **kwargs), t, True, 100+t)


def selected(mode='autonomous', separation=.2):
    guard = MotionGuard(separation)
    pad(guard, 0)
    pad(guard, .05, **({'rb': True} if mode == 'autonomous' else {'lb': True}))
    return guard


def request(guard, t=.06, forward=.05, yaw=0., stamp=None):
    guard.receive_request(forward, yaw, 100+t if stamp is None else stamp, t, 100+t)


def command(guard, t=.07):
    return guard.command(t, True, 100+t)


@pytest.mark.parametrize('mode', ['autonomous', 'manual'])
def test_requires_release_and_center_before_selection(mode):
    guard = MotionGuard(.2)
    buttons = {'rb': True} if mode == 'autonomous' else {'lb': True}
    pad(guard, 0, **buttons)
    assert not command(guard)[2]
    pad(guard, .08, left=.1)
    pad(guard, .09, **buttons)
    assert not command(guard, .1)[2]
    pad(guard, .11)
    pad(guard, .12, **buttons)
    request(guard, .13)
    assert command(guard, .14)[2]


@pytest.mark.parametrize('fault', [{'a': True, 'rb': True}, {'lb': True, 'rb': True},
                                  {'rb': True, 'valid': False}])
def test_latched_input_fault(fault):
    guard = selected()
    request(guard)
    assert command(guard)[0] > 0
    pad(guard, .08, **fault)
    assert command(guard, .09) == (0, 0, False)
    pad(guard, .1, rb=True)
    request(guard, .11)
    assert command(guard, .12) == (0, 0, False)
    pad(guard, .13)
    pad(guard, .14, rb=True)
    request(guard, .15)
    assert command(guard, .16)[0] > 0


def test_guard_uses_published_profile_not_raw_y_edges():
    guard = selected('manual')
    for i in range(1, 15):
        t = .05+i*.02
        pad(guard, t, lb=True, left=1, right=-1, y=True, y_presses=i)
        request(guard, t+.001, forward=1.)
        assert command(guard, t+.002) == (.15, -.15, True)
    pad(guard, .4)
    assert command(guard, .41) == (0, 0, False)


def test_direct_sampled_mode_change_requires_release():
    guard = selected()
    request(guard)
    pad(guard, .08, lb=True, left=1)
    assert command(guard, .09) == (0, 0, False)
    pad(guard, .1, lb=True)
    assert not command(guard, .11)[2]
    pad(guard, .12)
    pad(guard, .13, lb=True, left=1)
    assert command(guard, .14) == (.15, 0, True)


@pytest.mark.parametrize('forward,yaw,stamp', [
    (math.nan, 0, 100.06), (0, math.inf, 100.06),
    (.05, 0, 99.), (.05, 0, 101.),
    (.05, 0, math.nan),
])
def test_bad_requests_disarm(forward, yaw, stamp):
    guard = selected()
    request(guard, forward=forward, yaw=yaw, stamp=stamp)
    assert command(guard) == (0, 0, False)
    request(guard, .08)
    assert command(guard, .09) == (0, 0, False)


def test_spin_and_combined_limit():
    guard = selected()
    request(guard, forward=0, yaw=1.5)
    assert command(guard) == (-.15, .15, True)
    request(guard, .08, forward=.05, yaw=.5)
    assert command(guard, .09) == (0., .1, True)


@pytest.mark.parametrize('forward,yaw,expected', [
    (.151, 0, (.15, .15)), (-1, 0, (-.15, -.15)),
    (0, 3, (-.15, .15)), (0, -3, (.15, -.15)),
    (.1, 1, (0, .15)), (.2, 1, (.05, .15)),
    (-.2, -1, (-.05, -.15)), (.05, 2, (-.09, .15)),
    (1e308, 0, (.15, .15)),
])
def test_oversized_request_scales_together_without_disarming(forward, yaw, expected):
    guard = selected()
    request(guard, forward=forward, yaw=yaw)
    left, right, enabled = command(guard)
    assert enabled and guard.mode == 'autonomous'
    assert (left, right) == pytest.approx(expected)
    assert max(abs(left), abs(right)) <= TREAD_LIMIT
    assert guard.reason == 'autonomous (speed limited)'
    request(guard, .08, forward=.02, yaw=0)
    assert command(guard, .09) == (.02, .02, True)
    assert guard.reason == 'autonomous'
    assert not guard.request_limited


def test_repeated_clipping_preserves_timeout_and_rearm_requirements():
    guard = selected()
    for t in (.06, .16, .26):
        pad(guard, t, rb=True)
        request(guard, t, forward=1)
        assert command(guard, t+.001) == (.15, .15, True)
    pad(guard, .4, rb=True)
    pad(guard, .52, rb=True)
    assert command(guard, .52) == (0, 0, False)
    assert guard.reason == 'navigation request expired'
    request(guard, .53, forward=.01)
    assert command(guard, .54) == (0, 0, False)


def test_zero_and_exact_limit_do_not_need_scaling():
    guard = selected()
    request(guard, forward=0)
    assert command(guard) == (0, 0, True)
    assert not guard.request_limited
    request(guard, .08, forward=TREAD_LIMIT)
    assert command(guard, .09) == (TREAD_LIMIT, TREAD_LIMIT, True)
    assert not guard.request_limited


def test_nonfinite_conversion_remains_a_fault():
    guard = selected(separation=2)
    request(guard, forward=1e308, yaw=1e308)
    assert command(guard) == (0, 0, False)
    assert guard.reason == 'nonfinite tread conversion'


@pytest.mark.parametrize('stamp', [100.04, 100.05])
@pytest.mark.parametrize('forward', [0., .05])
def test_inflight_preselection_request_is_ignored_until_fresh_request(stamp, forward):
    guard = selected()
    request(guard, forward=forward, stamp=stamp)
    assert guard.mode == 'autonomous'
    assert guard.request is None
    assert guard.request_at == guard.request_stamp == -math.inf
    assert command(guard) == (0., 0., True)
    request(guard, .08)
    assert command(guard, .09) == (.05, .05, True)


def test_ignored_requests_do_not_extend_initial_deadline_or_rearm():
    guard = selected()
    for t in (.06, .15, .29):
        pad(guard, t, rb=True)
        request(guard, t, stamp=100.05)
        assert command(guard, t) == (0., 0., True)
    assert command(guard, .301) == (0., 0., False)
    request(guard, .31)
    assert command(guard, .32) == (0., 0., False)
    pad(guard, .33)
    pad(guard, .34, rb=True)
    request(guard, .35)
    assert command(guard, .36) == (.05, .05, True)


def test_preselection_request_after_first_accepted_request_disarms():
    guard = selected()
    request(guard)
    request(guard, .07, stamp=100.04)
    assert command(guard, .08) == (0., 0., False)
    request(guard, .09)
    assert command(guard, .1) == (0., 0., False)


def test_ignored_request_does_not_reuse_previous_modes_command():
    guard = selected()
    request(guard)
    pad(guard, .08)
    pad(guard, .09, rb=True)
    request(guard, .1, stamp=100.08)
    assert guard.request_at == .06
    assert command(guard, .11) == (0., 0., True)
    request(guard, .12, forward=.02)
    assert command(guard, .13) == (.02, .02, True)


def test_expiry_not_refreshed_by_joystick_or_restored_stream():
    guard = selected()
    request(guard)
    for t in (.15, .25, .35):
        pad(guard, t, rb=True)
        command(guard, t)
    assert command(guard, .36) == (0, 0, False)
    request(guard, .37)
    assert command(guard, .38) == (0, 0, False)


@pytest.mark.parametrize('initial', [True, False])
def test_late_callback_cannot_hide_timeout_between_ticks(initial):
    guard = selected()
    if not initial:
        request(guard)
    pad(guard, .2, rb=True)
    pad(guard, .32, rb=True)
    request(guard, .33)
    assert command(guard, .34) == (0, 0, False)


def test_first_request_deadline_and_previous_mode_request():
    guard = selected()
    assert command(guard) == (0, 0, True)
    pad(guard, .2, rb=True)
    assert command(guard, .301) == (0, 0, False)
    pad(guard, .31)
    pad(guard, .32, rb=True)
    request(guard, .33)
    assert command(guard, .34)[0] > 0
    pad(guard, .35)
    pad(guard, .36, rb=True)
    assert command(guard, .37) == (0, 0, True)


def test_replay_and_source_clock_jump_stop():
    guard = selected()
    request(guard)
    request(guard, .07, stamp=100.06)
    assert command(guard, .08) == (0, 0, False)
    guard = selected()
    request(guard)
    assert guard.command(.07, True, 99.) == (0, 0, False)


def test_health_restart_and_joystick_replay():
    guard = selected()
    request(guard)
    assert guard.command(.07, False, 100.07) == (0, 0, False)
    pad(guard, .08, rb=True)
    assert not command(guard, .09)[2]
    guard = selected()
    request(guard)
    guard.update(Pad('new-session', 1, rb=True), .08, True, 100.08)
    assert not command(guard, .09)[2]
    guard = selected()
    guard.update(Pad('session', 2, rb=True), .08, True, 100.08)
    assert not command(guard, .09)[2]


def test_unconfigured_conversion_only_accepts_zero():
    guard = selected(separation=0.)
    request(guard, forward=0.)
    assert command(guard) == (0, 0, True)
    request(guard, .08)
    assert command(guard, .09) == (0, 0, False)


@pytest.mark.parametrize('change', [
    {'command_session': 'old'}, {'seq': 2}, {'seq': 0}, {'input_age': .3},
    {'command_age': .3}, {'source_age': -.1}, {'left': .16}, {'right': math.nan},
    {'enabled': False}, {'motion_enabled': False}, {'health_ok': False},
    {'command_mode': 'manual'}, {'selected_mode': 'stopped'},
    {'source_stamp': 100.}, {'source_stamp': 99.}, {'source_stamp': math.nan},
    {'pad': Pad('session', 1)}, {'pad': Pad('session', 1, rb=True, lb=True)},
    {'pad': Pad('session', 1, rb=True, a=True)},
])
def test_independent_hardware_gate(change):
    args = dict(session='session', command_session='session', seq=1, input_seq=1,
                input_age=0., command_age=0., source_age=0., left=.1, right=.1,
                enabled=True, pad=Pad('session', 1, rb=True), health_ok=True,
                motion_enabled=True, command_mode='autonomous', selected_mode='autonomous',
                source_stamp=100.1, mode_entered=100.)
    assert validate_command(**args)
    args.update(change)
    assert not validate_command(**args)


@pytest.fixture
def navigation(monkeypatch):
    """Exercise the actual ROS adapter with minimal in-process message mocks."""
    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)

    class Node:
        def __init__(self, name):
            self.subscriptions = {}
            self.messages = []
        def declare_parameter(self, name, value):
            return types.SimpleNamespace(value=value)
        def create_subscription(self, kind, topic, callback, qos):
            self.subscriptions[topic] = callback
        def create_publisher(self, *args):
            return types.SimpleNamespace(publish=self.messages.append)
        def create_timer(self, period, callback):
            assert period == .05
        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: 'stamp'))
        def get_logger(self):
            return types.SimpleNamespace(error=lambda text: None)

    modules = {
        'rclpy': {}, 'rclpy.node': {'Node': Node},
        'rclpy.qos': {'qos_profile_sensor_data': 'sensor',
                     'QoSProfile': lambda **kw: kw,
                     'ReliabilityPolicy': types.SimpleNamespace(BEST_EFFORT='best_effort')},
        'rclpy.executors': {'ExternalShutdownException': type('Shutdown', (Exception,), {})},
        'sensor_msgs.msg': {'Image': Message, 'Imu': Message},
        'cv_bridge': {'CvBridge': lambda: types.SimpleNamespace(imgmsg_to_cv2=lambda msg, **kw: msg.data)},
        'course_robot_interfaces.msg': {'MotionRequest': Message},
    }
    for name, values in modules.items():
        mod = types.ModuleType(name)
        mod.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, mod)
    path = ROOT/'robot/src/team_navigation/team_navigation/navigation.py'
    spec = importlib.util.spec_from_file_location('project_navigation_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Navigation()


def test_starter_receives_sensors_and_publishes_zero(navigation):
    msg = types.SimpleNamespace(data=[[[1, 2, 3]]], header=types.SimpleNamespace(stamp='image-stamp'))
    navigation.subscriptions['camera/image_raw'](msg)
    navigation.subscriptions['imu/data'](msg)
    assert navigation.image == msg.data
    assert navigation.image_stamp == 'image-stamp'
    assert navigation.imu is msg
    for _ in range(20):
        navigation.tick()
    assert len(navigation.messages) == 20
    assert all(m.forward_speed_m_s == m.yaw_rate_rad_s == 0 and m.header.stamp == 'stamp'
               for m in navigation.messages)


def test_starter_exception_and_nonfinite_do_not_reuse_motion(navigation):
    navigation.compute_motion = lambda action: (.1, .2)
    navigation.tick()
    assert navigation.messages[-1].forward_speed_m_s == .1
    navigation.compute_motion = lambda action: (math.nan, 0)
    navigation.tick()
    assert navigation.messages[-1].forward_speed_m_s == 0
    def broken(_):
        raise RuntimeError('test')
    navigation.compute_motion = broken
    navigation.tick()
    assert navigation.messages[-1].yaw_rate_rad_s == 0


def test_navigation_smooths_new_offsets_and_rejects_stale_or_blue(navigation):
    navigation._now_seconds = lambda: 10.05
    navigation.image_stamp = types.SimpleNamespace(sec=10, nanosec=0)
    estimate = navigation.update_estimate(
        {'offset': .4, 'blue_fraction': 0., 'fork': False}, None)
    assert estimate['valid'] and estimate['offset'] == .4

    navigation._now_seconds = lambda: 10.15
    navigation.image_stamp = types.SimpleNamespace(sec=10, nanosec=100_000_000)
    estimate = navigation.update_estimate(
        {'offset': -.2, 'blue_fraction': 0., 'fork': False}, None)
    assert estimate['valid']
    assert estimate['offset'] == pytest.approx(.19)
    assert estimate['offset_rate'] == pytest.approx(-2.1)

    navigation._now_seconds = lambda: 10.31
    estimate = navigation.update_estimate(
        {'offset': -.2, 'blue_fraction': 0., 'fork': False}, None)
    assert not estimate['valid']

    navigation._now_seconds = lambda: 10.32
    navigation.image_stamp = types.SimpleNamespace(sec=10, nanosec=320_000_000)
    estimate = navigation.update_estimate(
        {'offset': .1, 'blue_fraction': navigation.blue_stop_fraction, 'fork': False}, None)
    assert not estimate['valid']


def test_navigation_slows_for_offset_and_pd_turns_toward_road(navigation):
    action = navigation.choose_action(
        {'valid': True, 'offset': .1, 'offset_rate': 0., 'fork': False})
    speed, yaw = navigation.compute_motion(action)
    assert navigation.min_speed < speed < navigation.max_speed
    assert yaw < 0

    sharp = navigation.choose_action(
        {'valid': True, 'offset': .8, 'offset_rate': 0., 'fork': False})
    assert navigation.compute_motion(sharp)[0] == navigation.min_speed

    fork = navigation.choose_action(
        {'valid': True, 'offset': 0., 'offset_rate': 0., 'fork': True})
    assert navigation.compute_motion(fork)[0] == navigation.fork_speed
    assert navigation.compute_motion(None) == (0., 0.)


def test_hardware_checks_mode_and_post_selection_stamp():
    text = (ROOT/'robot/src/a2r_robot/a2r_robot/hardware.py').read_text()
    assert 'selected_mode=self.local_deadman.mode' in text
    assert 'mode_entered=self.local_deadman.entered_ros' in text
    assert "pubs[0].node_name=='rvr_safety'" in text
