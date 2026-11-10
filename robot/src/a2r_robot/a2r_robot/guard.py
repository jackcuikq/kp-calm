"""ROS-independent mode selection, motion requests, and final hardware checks."""
from dataclasses import dataclass
import math

VERSION = '0.4.0'
SPEEDS = (.15, .60, 1.40)
PROFILE_NAMES = ('Low', 'Medium', 'High')
TREAD_LIMIT = SPEEDS[0]  # Startup profile; runtime limits come from the joystick.
TIMEOUT = .25
FUTURE_TOLERANCE = .05


@dataclass
class Pad:
    session: str
    sequence: int
    left: float = 0.
    right: float = 0.
    rb: bool = False
    a: bool = False
    y: bool = False
    y_presses: int = 0
    valid: bool = True
    lb: bool = False
    speed_profile: int = 0

    def sane(self):
        return (self.valid and bool(self.session) and self.sequence > 0 and
                type(self.speed_profile) is int and 0 <= self.speed_profile < len(SPEEDS) and
                all(math.isfinite(v) and abs(v) <= 1 for v in (self.left, self.right)))


class SpeedSelector:
    """Hardware-owned Y selection, replicated as state rather than button edges.

    Consume even rejected presses so they cannot take effect later. Requiring
    two neutral samples avoids selecting a higher limit on a release sample.
    """
    def __init__(self):
        self.profile = 0
        self.previous = None
        self.received = -math.inf

    @staticmethod
    def neutral(pad):
        return pad.sane() and not (pad.a or pad.lb or pad.rb) and pad.left == pad.right == 0

    def update(self, pad, now, health_ok):
        previous = self.previous
        if (previous is None or pad.session != previous.session or
                not pad.sane() or not health_ok or
                not 0 <= now-self.received <= TIMEOUT):
            self.profile = 0
            self.previous = pad if pad.sane() and health_ok else None
            self.received = now
            return self.profile
        if pad.sequence <= previous.sequence or pad.y_presses < previous.y_presses:
            self.profile = 0
            self.previous = None
        else:
            if (pad.y_presses-previous.y_presses == 1 and
                    self.neutral(previous) and self.neutral(pad)):
                self.profile = (self.profile+1) % len(SPEEDS)
            self.previous = pad
        self.received = now
        return self.profile


class Deadman:
    """Release-to-select gate, used independently by safety and hardware.

    Centered sticks and released bumpers arm selection, not motion.
    A held bumper never clears a latched fault.
    """
    def __init__(self):
        self.session = None
        self.pad = None
        self.received = -math.inf
        self.ready = False
        self.mode = 'stopped'
        self.entered = self.entered_ros = math.inf
        self.reason = 'release bumpers and center sticks'

    def disarm(self, reason):
        self.ready = False
        self.mode = 'stopped'
        self.reason = reason

    @property
    def speed_profile(self):
        return self.pad.speed_profile if self.pad and self.pad.sane() else 0

    @property
    def tread_limit(self):
        return SPEEDS[self.speed_profile]

    def update(self, pad, now, health_ok, ros_now=None):
        if pad.session != self.session:
            self.session = pad.session
            self.pad = None
            self.disarm('hardware session changed')
        if self.pad and pad.sequence <= self.pad.sequence:
            self.disarm('replayed/out-of-order joystick')
            return
        if now - self.received > TIMEOUT:
            self.disarm('joystick stale')
        profile_changed_while_active = bool(
            self.pad and pad.speed_profile != self.pad.speed_profile and
            self.mode != 'stopped' and (pad.lb or pad.rb))
        self.pad, self.received = pad, now
        if not pad.sane() or not health_ok:
            self.disarm('hardware/input unavailable')
        elif profile_changed_while_active:
            self.disarm('release bumpers after speed profile change')
        elif pad.a:
            self.disarm('A pressed')
        elif pad.lb and pad.rb:
            self.disarm('both bumpers held')
        elif not (pad.lb or pad.rb):
            self.mode = 'stopped'
            self.ready = pad.left == pad.right == 0
            self.reason = ('ready for deliberate selection' if self.ready else
                           'center sticks and release bumpers')
        else:
            selected = 'autonomous' if pad.rb else 'manual'
            if self.mode not in ('stopped', selected):
                self.disarm('release bumpers before changing modes')
            elif self.ready:
                if self.mode != selected:
                    self.entered = now
                    self.entered_ros = now if ros_now is None else ros_now
                self.mode = selected
                self.reason = selected

    def command(self, now, health_ok):
        if not health_ok or not 0 <= now - self.received <= TIMEOUT:
            self.disarm('hardware or joystick stale')
        p = self.pad
        enabled = bool(p and p.sane() and (p.rb != p.lb) and not p.a
                       and self.ready and self.mode != 'stopped' and health_ok)
        if not enabled:
            return 0., 0., False
        if self.mode == 'manual':
            return p.left*self.tread_limit, p.right*self.tread_limit, True
        return 0., 0., True


class MotionGuard(Deadman):
    """Use monotonic arrival age and ROS source age independently.

    Zero separation permits only zero requests until staff configure and
    check the approximate body-to-tread conversion on the actual platform.
    """
    def __init__(self, track_width_m=0.):
        super().__init__()
        if not math.isfinite(track_width_m) or track_width_m < 0:
            raise ValueError('track_width_m must be finite and nonnegative')
        self.separation = track_width_m
        self.request = None
        self.request_limited = False
        self.request_at = -math.inf
        self.request_stamp = -math.inf

    def receive_request(self, forward, yaw, stamp, now, ros_now):
        # A newly arriving packet must not conceal an expired previous request
        # or an elapsed initial wait between timer callbacks.
        self.command(now, True, ros_now)
        if self.mode != 'autonomous' or not self.ready:
            return
        if not all(math.isfinite(v) for v in (forward, yaw, stamp, now, ros_now)):
            self.disarm('nonfinite navigation request')
            return
        if not -FUTURE_TOLERANCE <= ros_now-stamp <= TIMEOUT:
            self.disarm('stale/future navigation request')
            return
        if now <= self.entered or stamp <= self.entered_ros:
            if self.request_at <= self.entered:
                # An in-flight request may arrive just after RB selection.
                # Ignore it only during the initial wait. Do not save it or
                # renew the deadline; command() above already checks expiry.
                return
            self.disarm('navigation request predates mode selection')
            return
        if self.request_at > self.entered and stamp <= self.request_stamp:
            self.disarm('replayed/out-of-order navigation request')
            return
        if self.separation == 0 and (forward != 0 or yaw != 0):
            self.disarm('autonomous conversion not configured')
            return
        left = forward - self.separation*yaw/2
        right = forward + self.separation*yaw/2
        if any(not math.isfinite(v) for v in (left, right)):
            self.disarm('nonfinite tread conversion')
            return
        peak = max(abs(left), abs(right))
        limit = self.tread_limit
        self.request_limited = peak > limit
        scale = limit / peak if self.request_limited else 1.
        # Scale together to preserve the requested tread ratio. Clamp only
        # floating-point roundoff before the independent hardware check.
        self.request = tuple(max(-limit, min(limit, v*scale))
                             for v in (left, right))
        self.request_stamp, self.request_at = stamp, now

    def command(self, now, health_ok, ros_now=None):
        manual = super().command(now, health_ok)
        if not manual[2] or self.mode != 'autonomous':
            return manual
        ros_now = now if ros_now is None else ros_now
        if self.request_at <= self.entered:
            if now-self.entered <= TIMEOUT:
                self.reason = 'waiting for post-selection navigation request'
                return 0., 0., True
            self.disarm('no navigation request after selection')
        elif (not 0 <= now-self.request_at <= TIMEOUT or
              not -FUTURE_TOLERANCE <= ros_now-self.request_stamp <= TIMEOUT):
            self.disarm('navigation request expired')
        else:
            self.reason = ('autonomous (speed limited)' if self.request_limited
                           else 'autonomous')
            return *self.request, True
        return 0., 0., False


def validate_command(session, command_session, seq, input_seq, input_age, command_age,
                     source_age, left, right, enabled, pad, health_ok, motion_enabled,
                     *, command_mode, selected_mode, source_stamp, mode_entered,
                     command_profile=0):
    """No command alone can override current bumpers, A, or the speed cap."""
    return bool(motion_enabled and health_ok and enabled and pad and pad.sane() and
                (pad.rb != pad.lb) and not pad.a and session == command_session == pad.session and
                command_mode == selected_mode == ('autonomous' if pad.rb else 'manual') and
                command_profile == pad.speed_profile and
                math.isfinite(source_stamp) and source_stamp > mode_entered and
                0 <= input_seq-seq <= 5 and 0 < seq <= input_seq and
                0 <= input_age <= TIMEOUT and 0 <= command_age <= TIMEOUT and
                -FUTURE_TOLERANCE <= source_age <= TIMEOUT and
                all(math.isfinite(v) and abs(v) <= SPEEDS[pad.speed_profile] for v in (left, right)))
