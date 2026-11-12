"""Record a short clip from the robot's ROS compressed-camera topic."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage


def parse_args(args):
    parser = argparse.ArgumentParser(
        description='Record a short MJPEG/AVI clip from the robot camera topic.')
    parser.add_argument(
        '--seconds', type=float, default=5.0,
        help='clip duration after the first camera frame arrives (default: 5)')
    parser.add_argument(
        '--namespace', default=os.environ.get('ROBOT_NAMESPACE', 'jetson1'),
        help='robot namespace (default: $ROBOT_NAMESPACE or jetson1)')
    parser.add_argument(
        '--output', type=Path,
        help='output .avi path (default: camera_clips/clip_TIMESTAMP.avi)')
    parsed = parser.parse_args(args)
    if not 0.5 <= parsed.seconds <= 60.0:
        parser.error('--seconds must be between 0.5 and 60')
    if parsed.output is not None and parsed.output.suffix.lower() != '.avi':
        parser.error('--output must end in .avi')
    return parsed


class CameraRecorder(Node):
    def __init__(self, namespace, output, seconds):
        super().__init__('camera_recorder')
        namespace = namespace.strip('/')
        if not namespace:
            raise ValueError('robot namespace must not be empty')
        self.topic = f'/{namespace}/camera/image_raw/compressed'
        self.output = output
        self.seconds = seconds
        self.writer = None
        self.started_at = None
        self.last_frame_at = None
        self.frames = 0
        self.done = False
        self.failed = None
        self.create_subscription(
            CompressedImage, self.topic, self.image_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'Waiting for {self.topic}; output will be {self.output}')

    def image_cb(self, msg):
        if self.done:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.failed = 'received a camera message that OpenCV could not decode'
            self.done = True
            return

        now = time.monotonic()
        if self.writer is None:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            height, width = frame.shape[:2]
            self.writer = cv2.VideoWriter(
                str(self.output), cv2.VideoWriter_fourcc(*'MJPG'), 30.0, (width, height))
            if not self.writer.isOpened():
                self.failed = f'could not open video writer for {self.output}'
                self.writer.release()
                self.writer = None
                self.done = True
                return
            self.started_at = now
            self.get_logger().info(
                f'Recording {width}x{height} for {self.seconds:.1f} seconds...')

        self.writer.write(frame)
        self.frames += 1
        self.last_frame_at = now
        if now - self.started_at >= self.seconds:
            self.done = True

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None


def main(args=None):
    rclpy.init(args=args)
    cli = parse_args(remove_ros_args(args=args)[1:])
    output = cli.output
    if output is None:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output = Path.cwd() / 'camera_clips' / f'clip_{stamp}.avi'
    else:
        output = output.expanduser().resolve()

    node = CameraRecorder(cli.namespace, output, cli.seconds)
    wait_started = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.started_at is None and now - wait_started > 10.0:
                node.failed = f'no frames received from {node.topic} within 10 seconds'
                node.done = True
            elif node.last_frame_at is not None and now - node.last_frame_at > 2.0:
                node.failed = 'camera frames stopped arriving during the recording'
                node.done = True
    except (KeyboardInterrupt, ExternalShutdownException):
        node.failed = 'recording interrupted'
    finally:
        node.close()
        frames = node.frames
        failure = node.failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failure:
        raise RuntimeError(failure)
    print(f'Saved {frames} frames to {output}')


if __name__ == '__main__':
    main()
