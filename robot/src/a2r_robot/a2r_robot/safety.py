import math
import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from a2r_robot_interfaces.msg import JoystickState, HardwareStatus, TankCommand
from course_robot_interfaces.msg import MotionRequest
from .guard import MotionGuard, Pad, TIMEOUT, PROFILE_NAMES, VERSION

CONTROL_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)


def stamp_age(stamp, node):
    return (node.get_clock().now().nanoseconds - stamp.sec*10**9-stamp.nanosec)/1e9


class Safety(Node):
    def __init__(self):
        super().__init__('rvr_safety')
        separation = self.declare_parameter('track_width_m', 0.).value
        self.guard = MotionGuard(separation)
        self.health = None
        self.health_at = -math.inf
        self.pub = self.create_publisher(TankCommand, 'motors/command', CONTROL_QOS)
        self.status_pub = self.create_publisher(String, 'safety/status', 1)
        self.create_subscription(HardwareStatus, 'hardware/status', self.health_cb, CONTROL_QOS)
        self.create_subscription(JoystickState, 'joystick/state', self.joy_cb, CONTROL_QOS)
        self.create_subscription(MotionRequest, 'navigation/request', self.request_cb, CONTROL_QOS)
        self.create_timer(.05, self.tick)
        self.last_reason = None

    def ros_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def request_cb(self, msg):
        self.guard.receive_request(msg.forward_speed_m_s, msg.yaw_rate_rad_s,
                                   msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
                                   time.monotonic(), self.ros_seconds())
        self.tick()

    def healthy(self):
        h = self.health
        return bool(h and time.monotonic()-self.health_at <= TIMEOUT and
                    h.uart_ok and h.imu_fresh and h.encoders_fresh and
                    h.joystick_connected and not h.demo_locked)

    def health_cb(self, msg):
        if time.monotonic()-self.health_at > TIMEOUT:
            self.guard.disarm('hardware status gap')
        if not -.05 <= stamp_age(msg.header.stamp, self) <= TIMEOUT:
            self.guard.disarm('stale hardware status')
            self.health = None
            return
        if self.health and self.health.hardware_session != msg.hardware_session:
            self.guard.disarm('hardware restarted')
        self.health, self.health_at = msg, time.monotonic()
        if not self.healthy():
            self.guard.disarm('hardware fault')

    def joy_cb(self, msg):
        axes, buttons = msg.joy.axes, msg.joy.buttons
        valid = (msg.connected and not msg.events_lost and len(axes) >= 4 and len(buttons) >= 6
                 and -.05 <= stamp_age(msg.joy.header.stamp, self) <= TIMEOUT)
        p = Pad(msg.hardware_session, msg.sequence,
                axes[1] if len(axes)>1 else 0., axes[3] if len(axes)>3 else 0.,
                bool(buttons[5]) if len(buttons)>5 else False,
                bool(buttons[0]) if buttons else False,
                bool(buttons[3]) if len(buttons)>3 else False,
                msg.y_presses, valid, bool(buttons[4]) if len(buttons)>4 else False,
                msg.speed_profile)
        self.guard.update(p, time.monotonic(), self.healthy() and
                          bool(self.health and self.health.hardware_session == p.session),
                          self.ros_seconds())
        # Release/A/input fault propagates without waiting for the 20 Hz timer.
        self.tick()

    def tick(self):
        now = time.monotonic()
        left, right, enabled = self.guard.command(now, self.healthy(), self.ros_seconds())
        msg = TankCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hardware_session = self.guard.session or ''
        msg.joystick_sequence = self.guard.pad.sequence if self.guard.pad else 0
        msg.left_m_s, msg.right_m_s, msg.enabled = left, right, enabled
        msg.control_mode = self.guard.mode
        msg.speed_profile = self.guard.speed_profile
        self.pub.publish(msg)
        age = now-self.guard.request_at
        text = (f'{self.guard.reason}; version={VERSION}; mode={self.guard.mode}; '
                f'speed_profile={PROFILE_NAMES[self.guard.speed_profile]}; '
                f'tread_limit={self.guard.tread_limit:.2f} m/s; separation={self.guard.separation:.3f} m; '
                f'request_age={age:.3f} s; commanded_treads=({left:.3f},{right:.3f}) m/s')
        self.status_pub.publish(String(data=text))
        status_key = (self.guard.reason, self.guard.speed_profile)
        if status_key != self.last_reason:
            self.get_logger().info(text)
            self.last_reason = status_key


def main():
    # Keep context teardown out of the signal handler: Ctrl+C can reach both
    # launch and its child, and must not invalidate an in-flight subscription.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Safety()
    stopping = False
    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.guard.disarm('safety node stopping')
        if rclpy.ok():
            node.tick()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
