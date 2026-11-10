#!/usr/bin/env python3
"""Publish images as if they came from the robot camera (ROS, no base launch needed).

Feeds <ns>/camera/image_raw (bgr8) and an empty <ns>/imu/data at 20 Hz from
a folder of JPEG/PNG frames or a video file, so the real team launch can be
run on the Jetson (or any ROS 2 Jazzy machine) without the car:

    # terminal 1 (prepared):  python tools/fake_camera.py $ROBOT_NAMESPACE ~/frames
    # terminal 2 (prepared):  ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
    # terminal 3:             ros2 topic echo /$ROBOT_NAMESPACE/navigation/request --qos-reliability best_effort

Do not run this while the base launch is running (two publishers on the
camera topic). It is open-loop: the pictures do not react to the commands;
for closed loop use tools/sim.py on a laptop.
"""
import argparse
import sys
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class FakeCamera(Node):
    def __init__(self, ns, source, rate, loop, size):
        super().__init__('fake_camera')
        self.image_pub = self.create_publisher(Image, f'/{ns}/camera/image_raw', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, f'/{ns}/imu/data', qos_profile_sensor_data)
        self.loop, self.size = loop, size
        p = Path(source)
        if p.is_dir():
            self.files = sorted(q for q in p.iterdir() if q.suffix.lower() in ('.jpg', '.jpeg', '.png'))
            if not self.files:
                raise SystemExit(f'no images in {p}')
            self.cap = None
        else:
            self.cap = cv2.VideoCapture(str(p))
            if not self.cap.isOpened():
                raise SystemExit(f'cannot open {p}')
            self.files = None
        self.i = 0
        self.create_timer(1.0 / rate, self.tick)

    def next_frame(self):
        if self.files is not None:
            if self.i >= len(self.files):
                if not self.loop:
                    return None
                self.i = 0
            img = cv2.imread(str(self.files[self.i])); self.i += 1
            return img
        ok, img = self.cap.read()
        if not ok:
            if not self.loop:
                return None
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, img = self.cap.read()
        return img if ok else None

    def tick(self):
        img = self.next_frame()
        if img is None:
            self.get_logger().info('end of frames'); rclpy.shutdown(); return
        if self.size:
            img = cv2.resize(img, self.size)
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, 'fake_camera'
        msg.height, msg.width = img.shape[:2]
        msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, img.shape[1] * 3
        msg.data = img.tobytes()
        self.image_pub.publish(msg)
        imu = Imu(); imu.header.stamp = stamp
        self.imu_pub.publish(imu)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('namespace')
    ap.add_argument('source', help='folder of images or a video file')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--once', action='store_true', help='stop at the end instead of looping')
    ap.add_argument('--size', default='1280x720', help='resize to WxH like the real camera; "" keeps original')
    a = ap.parse_args()
    size = tuple(int(s) for s in a.size.split('x')) if a.size else None
    rclpy.init()
    node = FakeCamera(a.namespace, a.source, a.rate, not a.once, size)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
