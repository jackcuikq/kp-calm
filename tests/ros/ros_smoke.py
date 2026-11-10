"""Run on ROS Jazzy without a car, in an isolated local test domain.

This exercises real generated messages and Safety callbacks, not motor hardware.
Run only with ROS_DOMAIN_ID=92 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST.
"""
import os
import time

if (os.environ.get('ROS_DOMAIN_ID') != '92' or
        os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST' or
        os.environ.get('CYCLONEDDS_URI')):
    raise SystemExit('Use isolated domain 92, LOCALHOST discovery, and unset CYCLONEDDS_URI.')

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from a2r_robot.safety import Safety, CONTROL_QOS
from a2r_robot_interfaces.msg import HardwareStatus, JoystickState, TankCommand
from course_robot_interfaces.msg import MotionRequest


def main():
    rclpy.init(args=['--ros-args', '-p', 'track_width_m:=0.2'])
    fixture = Node('course_smoke_fixture')
    safety = Safety()
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)
    executor.add_node(safety)
    hp = fixture.create_publisher(HardwareStatus, 'hardware/status', CONTROL_QOS)
    jp = fixture.create_publisher(JoystickState, 'joystick/state', CONTROL_QOS)
    np = fixture.create_publisher(MotionRequest, 'navigation/request', CONTROL_QOS)
    output = []
    fixture.create_subscription(TankCommand, 'motors/command', output.append, CONTROL_QOS)
    sequence = 0

    def run(seconds, rb=False, lb=False, a=False, nav=False, left=0.):
        nonlocal sequence
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            sequence += 1
            stamp = fixture.get_clock().now().to_msg()
            h = HardwareStatus()
            h.header.stamp = stamp
            h.hardware_session = 'software-test'
            h.uart_ok = h.imu_fresh = h.encoders_fresh = h.joystick_connected = True
            hp.publish(h)
            j = JoystickState()
            j.hardware_session, j.sequence, j.connected = 'software-test', sequence, True
            j.joy.header.stamp = stamp
            j.joy.axes = [0., left, 0., 0.]
            j.joy.buttons = [int(a), 0, 0, 0, int(lb), int(rb)]
            jp.publish(j)
            # Process selection before generating a post-selection request.
            for _ in range(8):
                executor.spin_once(timeout_sec=.002)
            if nav:
                n = MotionRequest()
                n.header.stamp = fixture.get_clock().now().to_msg()
                n.forward_speed_m_s = .05
                np.publish(n)
            for _ in range(8):
                executor.spin_once(timeout_sec=.002)
        assert output, 'No safety output received'
        return output[-1]

    try:
        # Discovery and neutral input before any selection.
        m = run(1.)
        assert not m.enabled
        m = run(.2, lb=True, left=1.)
        assert m.control_mode == 'manual' and abs(m.left_m_s-.15) < 1e-9
        m = run(.2, lb=True, a=True, left=1.)
        assert not m.enabled and m.left_m_s == 0.
        run(.2)
        m = run(.2, rb=True, nav=True)
        assert m.control_mode == 'autonomous' and abs(m.left_m_s-.05) < 1e-9
        m = run(.4, rb=True)
        assert not m.enabled
        m = run(.2, rb=True, nav=True)
        assert not m.enabled, 'Stream recovery restarted motion without release'
        run(.2)
        m = run(.2, rb=True, nav=True)
        assert m.enabled
        m = run(.2, rb=True, lb=True, nav=True)
        assert not m.enabled
        print('PASS: generated messages, pub/sub, mode selection, timeout, rearm')
    finally:
        executor.shutdown()
        fixture.destroy_node()
        safety.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
