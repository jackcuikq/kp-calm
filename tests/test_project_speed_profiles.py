"""Hardware-free global profile selection and independent gate checks."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'robot/src/a2r_robot'))
from a2r_robot.guard import Pad, SpeedSelector, MotionGuard, SPEEDS, validate_command


def test_y_cycles_once_per_press_and_returns_to_low():
    selector = SpeedSelector()
    p = Pad('car', 1)
    assert selector.update(p, 0, True) == 0
    for i in range(1, 7):
        p = replace(p, sequence=p.sequence+1, y=True, y_presses=i)
        assert selector.update(p, i*.1, True) == i % 3
        # A held key does not cycle again. Neither does its release.
        p = replace(p, sequence=p.sequence+1)
        assert selector.update(p, i*.1+.01, True) == i % 3
        p = replace(p, sequence=p.sequence+1, y=False)
        assert selector.update(p, i*.1+.02, True) == i % 3


@pytest.mark.parametrize('held', [{'rb': True}, {'lb': True}, {'a': True},
                                 {'left': .1}, {'right': -.1}])
def test_y_while_driving_or_non_neutral_is_consumed_not_deferred(held):
    selector = SpeedSelector()
    p = Pad('car', 1)
    selector.update(p, 0, True)
    p = replace(p, sequence=2, **held)
    selector.update(p, .05, True)
    p = replace(p, sequence=3, y=True, y_presses=1)
    assert selector.update(p, .1, True) == 0
    p = Pad('car', 4, y_presses=1)
    assert selector.update(p, .15, True) == 0
    p = replace(p, sequence=5, y=True, y_presses=2)
    assert selector.update(p, .2, True) == 1


def test_press_on_release_sample_is_ignored():
    selector = SpeedSelector()
    selector.update(Pad('car', 1, rb=True), 0, True)
    assert selector.update(Pad('car', 2, y_presses=1), .05, True) == 0
    assert selector.update(Pad('car', 3, y_presses=1), .1, True) == 0


def test_fault_gap_restart_and_missed_edges_do_not_raise_speed():
    selector = SpeedSelector()
    selector.update(Pad('car', 1), 0, True)
    assert selector.update(Pad('car', 2, y_presses=1), .05, True) == 1
    assert selector.update(Pad('car', 3, y_presses=4), .1, True) == 1
    assert selector.update(Pad('car', 4, valid=False, y_presses=5), .15, True) == 0
    assert selector.update(Pad('car', 5, y_presses=5), .2, True) == 0
    assert selector.update(Pad('car', 6, y_presses=6), .25, True) == 1
    assert selector.update(Pad('car', 7, y_presses=7), .6, True) == 0
    assert selector.update(Pad('car', 8, y_presses=8), .65, True) == 1
    assert selector.update(Pad('new-car-session', 1, y_presses=9), .7, True) == 0
    assert selector.update(Pad('new-car-session', 2, y_presses=10), .75, False) == 0


@pytest.mark.parametrize('profile,limit', list(enumerate(SPEEDS)))
@pytest.mark.parametrize('mode', ['manual', 'autonomous'])
def test_selected_limit_is_global_and_survives_dropped_joystick_packets(profile, limit, mode):
    guard = MotionGuard(.2)
    guard.update(Pad('car', 1, speed_profile=profile), 0, True, 100.)
    pad = Pad('car', 4, left=1, right=-.5, lb=mode=='manual', rb=mode=='autonomous',
              speed_profile=profile)
    guard.update(pad, .05, True, 100.05)
    if mode == 'autonomous':
        guard.receive_request(1., 5., 100.06, .06, 100.06)
    left, right, enabled = guard.command(.07, True, 100.07)
    assert enabled
    assert max(abs(left), abs(right)) <= limit
    assert (left, right) == pytest.approx((limit, -.5*limit) if mode=='manual'
                                         else (limit/3, limit))
    args = dict(session='car', command_session='car', seq=4, input_seq=4,
                input_age=.02, command_age=0, source_age=0, left=left, right=right,
                enabled=True, pad=pad, health_ok=True, motion_enabled=True,
                command_mode=mode, selected_mode=mode, source_stamp=100.07,
                mode_entered=100.05, command_profile=profile)
    assert validate_command(**args)
    assert not validate_command(**dict(args, left=limit+.001))
    assert not validate_command(**dict(args, command_profile=(profile+1)%3))
    assert not validate_command(**dict(args, right=math.nan))


def test_profile_change_without_neutral_sample_disarms():
    guard = MotionGuard(.2)
    guard.update(Pad('car', 1), 0, True)
    guard.update(Pad('car', 2, lb=True, left=1), .05, True)
    guard.update(Pad('car', 3, lb=True, left=1, speed_profile=1), .1, True)
    assert guard.command(.11, True) == (0, 0, False)
    guard.update(Pad('car', 4, speed_profile=1), .15, True)
    guard.update(Pad('car', 5, lb=True, left=1, speed_profile=1), .2, True)
    assert guard.command(.21, True) == (.6, 0, True)


@pytest.mark.parametrize('profile', [-1, 3, 100, float('nan')])
def test_invalid_profile_fails_closed(profile):
    guard = MotionGuard(.2)
    guard.update(Pad('car', 1), 0, True)
    guard.update(Pad('car', 2, lb=True, speed_profile=profile), .05, True)
    assert guard.command(.06, True) == (0, 0, False)


def test_base_adapters_carry_hardware_owned_profile():
    root = Path(__file__).resolve().parents[1]/'robot/src'
    for message in ('JoystickState', 'TankCommand'):
        assert 'uint8 speed_profile' in (root/f'a2r_robot_interfaces/msg/{message}.msg').read_text()
    hw = (root/'a2r_robot/a2r_robot/hardware.py').read_text()
    safety = (root/'a2r_robot/a2r_robot/safety.py').read_text()
    assert 'self.speed_selector.update(self.pad, now, self.healthy())' in hw
    assert 'command_profile=c.speed_profile' in hw
    assert 'msg.speed_profile = self.guard.speed_profile' in safety
