"""Yellow-road visual follower; the base safety node gates all motor commands."""
import math

import rclpy
from cv_bridge import CvBridge
from course_robot_interfaces.msg import MotionRequest
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.max_speed = float(self.declare_parameter(
            'requested_forward_speed_m_s', 0.05).value)
        self.min_speed = float(self.declare_parameter('minimum_forward_speed_m_s', 0.02).value)
        self.fork_speed = float(self.declare_parameter('fork_forward_speed_m_s', 0.035).value)
        self.slow_offset = float(self.declare_parameter('slow_offset_fraction', 0.35).value)
        self.smoothing_alpha = float(self.declare_parameter('smoothing_alpha', 0.35).value)
        self.max_image_age = float(self.declare_parameter('max_image_age_s', 0.20).value)
        self.kp = float(self.declare_parameter('steering_kp', 0.8).value)
        self.kd = float(self.declare_parameter('steering_kd', 0.05).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate_rad_s', 0.8).value)
        self.blue_stop_fraction = float(self.declare_parameter('blue_stop_fraction', 0.02).value)
        self.fork_min_fraction = float(self.declare_parameter('fork_min_fraction', 0.015).value)
        self.fork_min_separation = float(self.declare_parameter(
            'fork_min_separation_fraction', 0.20).value)
        self.yellow_lower = self._hsv_triplet(
            self.declare_parameter('yellow_hsv_lower', [18, 80, 80]).value)
        self.yellow_upper = self._hsv_triplet(
            self.declare_parameter('yellow_hsv_upper', [38, 255, 255]).value)
        self.blue_lower = self._hsv_triplet(
            self.declare_parameter('blue_hsv_lower', [90, 80, 50]).value)
        self.blue_upper = self._hsv_triplet(
            self.declare_parameter('blue_hsv_upper', [135, 255, 255]).value)
        self._validate_parameters()

        self.bridge = CvBridge()
        self.image = self.image_stamp = self.imu = None
        self._smoothed_offset = None
        self._offset_rate = 0.0
        self._last_frame_stamp = None
        self._last_valid_offset_at = None
        self._last_error = None
        self.create_subscription(Image, 'camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data', self.imu_cb, qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            MotionRequest, 'navigation/request',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.05, self.tick)

    @staticmethod
    def _hsv_triplet(value):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError('HSV bounds must contain three values')
        triplet = tuple(int(v) for v in value)
        if not 0 <= triplet[0] <= 179 or any(not 0 <= v <= 255 for v in triplet[1:]):
            raise ValueError('HSV bound is outside OpenCV HSV ranges')
        return triplet

    def _validate_parameters(self):
        positive = (self.max_speed, self.min_speed, self.fork_speed, self.slow_offset,
                    self.max_image_age, self.max_yaw_rate)
        if not all(math.isfinite(value) and value > 0 for value in positive):
            raise ValueError('speed, offset, age and yaw parameters must be finite and positive')
        if not self.min_speed <= self.fork_speed <= self.max_speed:
            raise ValueError('fork speed must be between minimum and requested speed')
        if not 0 < self.smoothing_alpha <= 1:
            raise ValueError('smoothing_alpha must be in (0, 1]')
        if not all(math.isfinite(value) and value >= 0 for value in (self.kp, self.kd)):
            raise ValueError('steering gains must be finite and nonnegative')
        if not 0 < self.blue_stop_fraction <= 1 or not 0 < self.fork_min_fraction <= 1:
            raise ValueError('pixel fractions must be in (0, 1]')
        if not 0 < self.fork_min_separation < 1:
            raise ValueError('fork_min_separation_fraction must be in (0, 1)')
        if any(low > high for low, high in zip(self.yellow_lower, self.yellow_upper)):
            raise ValueError('yellow HSV lower bound must not exceed upper bound')
        if any(low > high for low, high in zip(self.blue_lower, self.blue_upper)):
            raise ValueError('blue HSV lower bound must not exceed upper bound')

    @staticmethod
    def _stamp_seconds(stamp):
        if stamp is None or not hasattr(stamp, 'sec') or not hasattr(stamp, 'nanosec'):
            return None
        seconds = float(stamp.sec) + float(stamp.nanosec) / 1e9
        return seconds if math.isfinite(seconds) else None

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def image_cb(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_stamp = msg.header.stamp
        except Exception as exc:
            self.image = self.image_stamp = None
            self.get_logger().error(f'Image conversion failed: {exc}')

    def imu_cb(self, msg):
        self.imu = msg

    def process_image(self, image):
        """Return road-center offset in image-width fractions and blue/fork cues."""
        stamp = self._stamp_seconds(self.image_stamp)
        if image is None or stamp is None:
            return None
        now = self._now_seconds()
        if not -0.05 <= now - stamp <= self.max_image_age:
            return None

        import cv2

        if len(image.shape) != 3 or image.shape[2] != 3:
            raise ValueError('camera frame must be a BGR image with three channels')
        height, width = image.shape[:2]
        if height < 4 or width < 4:
            raise ValueError('camera frame is too small to process')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        blue_roi = blue[height // 2:, width // 6:width - width // 6]
        blue_fraction = cv2.countNonZero(blue_roi) / blue_roi.size
        if blue_fraction >= self.blue_stop_fraction:
            return {'blue_fraction': blue_fraction, 'offset': None, 'fork': False}

        row_top, row_bottom = int(height * 0.72), max(int(height * 0.82), int(height * 0.72) + 1)
        near_road = yellow[row_top:row_bottom, :]
        moments = cv2.moments(near_road, binaryImage=True)
        if moments['m00'] == 0:
            return {'blue_fraction': blue_fraction, 'offset': None, 'fork': False}
        near_center_x = moments['m10'] / moments['m00']

        look_top, look_bottom = int(height * 0.50), max(int(height * 0.75), int(height * 0.50) + 1)
        lookahead = yellow[look_top:look_bottom, :]
        half = width // 2
        left_roi, right_roi = lookahead[:, :half], lookahead[:, half:]
        left_count, right_count = cv2.countNonZero(left_roi), cv2.countNonZero(right_roi)
        left_moments = cv2.moments(left_roi, binaryImage=True)
        right_moments = cv2.moments(right_roi, binaryImage=True)
        left_center = (left_moments['m10'] / left_moments['m00']
                       if left_moments['m00'] else None)
        right_center = (half + right_moments['m10'] / right_moments['m00']
                        if right_moments['m00'] else None)

        half_area = lookahead.shape[0] * half
        is_fork = bool(
            left_center is not None and right_center is not None and
            left_count / half_area >= self.fork_min_fraction and
            right_count / half_area >= self.fork_min_fraction and
            right_center - left_center >= width * self.fork_min_separation)
        target_x = near_center_x
        if is_fork:
            target_x = left_center if left_count > right_count else right_center
        offset = (target_x - (width - 1) / 2) / (width / 2)
        return {'blue_fraction': blue_fraction, 'offset': offset, 'fork': is_fork}

    def update_estimate(self, observation, imu):
        """Smooth each new valid frame; offset is positive when the road is right."""
        del imu
        stamp = self._stamp_seconds(self.image_stamp)
        if observation is None or stamp is None:
            self._smoothed_offset = None
            self._offset_rate = 0.0
            self._last_valid_offset_at = None
            return {'valid': False, 'reason': 'missing or stale image'}
        if observation['blue_fraction'] >= self.blue_stop_fraction:
            self._smoothed_offset = None
            self._offset_rate = 0.0
            self._last_valid_offset_at = None
            return {'valid': False, 'reason': 'blue region in forward image'}
        raw_offset = observation['offset']
        if raw_offset is None:
            self._smoothed_offset = None
            self._offset_rate = 0.0
            self._last_valid_offset_at = None
            return {'valid': False, 'reason': 'yellow road not detected'}

        if stamp != self._last_frame_stamp:
            previous_offset = self._smoothed_offset
            previous_stamp = self._last_frame_stamp
            self._smoothed_offset = (raw_offset if previous_offset is None else
                                     self.smoothing_alpha * raw_offset +
                                     (1 - self.smoothing_alpha) * previous_offset)
            self._offset_rate = (
                (self._smoothed_offset - previous_offset) / (stamp - previous_stamp)
                if previous_offset is not None and previous_stamp is not None and
                stamp > previous_stamp else 0.0)
            self._last_frame_stamp = stamp
        self._last_valid_offset_at = stamp
        if self._now_seconds() - stamp > self.max_image_age:
            self._smoothed_offset = None
            self._offset_rate = 0.0
            self._last_valid_offset_at = None
            return {'valid': False, 'reason': 'yellow detection expired'}
        return {
            'valid': True,
            'offset': self._smoothed_offset,
            'offset_rate': self._offset_rate,
            'fork': observation['fork'],
        }

    def choose_action(self, estimate):
        """Slow for larger offsets; select the wider detected fork branch."""
        if not estimate['valid']:
            return None
        offset = abs(estimate['offset'])
        speed_fraction = max(0.0, 1.0 - offset / self.slow_offset)
        speed = self.min_speed + (self.max_speed - self.min_speed) * speed_fraction
        if estimate['fork']:
            speed = min(speed, self.fork_speed)
        return {'offset': estimate['offset'], 'offset_rate': estimate['offset_rate'],
                'speed': speed}

    def compute_motion(self, action):
        """Return forward m/s and left-turn rad/s; a missing action means stop."""
        if action is None:
            return 0.0, 0.0
        yaw_rate = -(self.kp * action['offset'] + self.kd * action['offset_rate'])
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        return action['speed'], yaw_rate

    def tick(self):
        forward, yaw = 0.0, 0.0
        try:
            observation = self.process_image(self.image)
            estimate = self.update_estimate(observation, self.imu)
            action = self.choose_action(estimate)
            forward, yaw = self.compute_motion(action)
            if not all(math.isfinite(value) for value in (forward, yaw)):
                raise ValueError('controller returned a nonfinite command')
            self._last_error = None
        except Exception as exc:
            forward, yaw = 0.0, 0.0
            error = str(exc)
            if error != self._last_error:
                self.get_logger().error(f'Navigation failed; requesting zero motion: {error}')
                self._last_error = error
        msg = MotionRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.forward_speed_m_s = float(forward)
        msg.yaw_rate_rad_s = float(yaw)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = Navigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
