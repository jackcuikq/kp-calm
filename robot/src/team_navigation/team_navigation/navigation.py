"""Receive sensors and request motion through the supplied safety node.

Pipeline (20 Hz, see tick):
    process_image   -> observation  (perception, Jeff)
    update_estimate -> estimate     (smoothing + staleness, Jeff)
    choose_action   -> action       (follow / stop, Claudia)
    compute_motion  -> (v, yaw)     (PD controller, Youran)

Data contract between hooks (all plain dicts, pixel units unless noted):
    observation = {'offset_px': float, 'width_px': float, 'valid': bool, 'stamp': Time}
                  offset_px > 0 means the road centre is to the RIGHT of image centre.
    estimate    = {'offset_px': float, 'd_offset_px_s': float, 'valid': bool}
    action      = {'kind': 'follow' | 'stop', 'offset_px': float, 'd_offset_px_s': float}
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from course_robot_interfaces.msg import MotionRequest


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        p = self.declare_parameter
        self.target_speed = p('requested_forward_speed_m_s', .05).value
        # --- control (config/navigation.yaml) ---
        self.kp = p('kp_rad_s_per_px', 0.02).value        # yaw per pixel of offset
        self.kd = p('kd_rad_per_px', 0.002).value          # yaw per (pixel/second)
        self.yaw_max = p('yaw_max_rad_s', 1.5).value
        self.v_min_frac = p('v_min_frac', 0.3).value       # slowest fraction of target speed in a turn
        self.half_width_px = p('image_half_width_px', 80.0).value  # half of the processed image width
        self.slow_gain = p('slow_gain', 1.0).value         # how hard offset reduces speed
        # --- estimation ---
        self.max_obs_age = p('max_observation_age_s', 0.2).value
        self.max_missing = p('max_missing_ticks', 3).value
        self.alpha = p('ema_alpha', 0.5).value
        # --- debugging: pretend perception sees a constant offset (dry run only) ---
        self.debug_fixed_offset = p('debug_fixed_offset_px', float('nan')).value

        self.bridge = CvBridge()
        self.image = self.image_stamp = self.imu = None
        self.previous_error = None
        # estimator state
        self.offset = 0.0
        self.d_offset = 0.0
        self.prev_stamp = None
        self.missing = self.max_missing  # start "lost" until the first valid observation
        self.create_subscription(Image, 'camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data', self.imu_cb, qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            MotionRequest, 'navigation/request',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.05, self.tick)

    def image_cb(self, msg):
        try:
            # OpenCV uses BGR channel order. The image is an H x W x 3 array.
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_stamp = msg.header.stamp
        except Exception as exc:
            self.image = self.image_stamp = None
            self.get_logger().error(f'Image conversion failed: {exc}')

    def imu_cb(self, msg):
        self.imu = msg

    # ------------------------------------------------------------------ perception
    def process_image(self, image):
        """Estimate road position from a BGR image; return None if unknown.

        TODO (perception): resize to ~160x120, HSV threshold on the road colour,
        take a band of rows near the bottom, return the road centre offset
        from image centre in pixels of the resized image, plus its width.
        """
        if not math.isnan(self.debug_fixed_offset):
            # Dry-run aid: constant observation so control can be tested alone.
            return {'offset_px': self.debug_fixed_offset, 'width_px': 60.0,
                    'valid': True, 'stamp': self.get_clock().now()}
        if image is None or self.image_stamp is None:
            return None
        return None

    # ------------------------------------------------------------------ estimation
    def update_estimate(self, observation, imu):
        """Smooth the offset and track how long since we last saw the road.

        IMU is not used for now (road-relative estimate only). A road-relative
        offset is not a global position.
        """
        now = self.get_clock().now()
        fresh = False
        if observation is not None and observation.get('valid'):
            stamp = observation['stamp']
            if not isinstance(stamp, Time):
                stamp = Time.from_msg(stamp)
            age = (now - stamp).nanoseconds * 1e-9
            fresh = age <= self.max_obs_age
        if not fresh:
            self.missing += 1
            self.d_offset = 0.0
            return {'offset_px': self.offset, 'd_offset_px_s': 0.0,
                    'valid': self.missing < self.max_missing}
        raw = float(observation['offset_px'])
        if self.missing >= self.max_missing:
            # Re-acquired after being lost: reset rather than blend with stale state.
            self.offset, self.d_offset = raw, 0.0
        else:
            smoothed = self.alpha * raw + (1.0 - self.alpha) * self.offset
            dt = (stamp - self.prev_stamp).nanoseconds * 1e-9 if self.prev_stamp is not None else 0.0
            self.d_offset = (smoothed - self.offset) / dt if dt > 1e-3 else 0.0
            self.offset = smoothed
        self.prev_stamp = stamp
        self.missing = 0
        return {'offset_px': self.offset, 'd_offset_px_s': self.d_offset, 'valid': True}

    # ------------------------------------------------------------------ planning
    def choose_action(self, estimate):
        """Choose a desired behavior, such as stop or follow the road."""
        if estimate is None or not estimate['valid']:
            return {'kind': 'stop', 'offset_px': 0.0, 'd_offset_px_s': 0.0}
        return {'kind': 'follow', 'offset_px': estimate['offset_px'],
                'd_offset_px_s': estimate['d_offset_px_s']}

    # ------------------------------------------------------------------ control
    def compute_motion(self, action):
        """Return (forward m/s, left-turn rad/s); no action means zero motion.

        PD on the lateral pixel offset. Positive yaw turns LEFT, positive
        offset means the road is to the RIGHT, hence the minus sign.
        Safety converts (v, w) to treads v -/+ b*w/2 with b = track_width_m and
        scales both down together if a tread exceeds the Y speed profile.
        """
        if action is None or action['kind'] != 'follow':
            return 0., 0.
        e = action['offset_px']
        de = action['d_offset_px_s']
        yaw = -(self.kp * e + self.kd * de)
        yaw = max(-self.yaw_max, min(self.yaw_max, yaw))
        # Slow down in proportion to how far off-centre we are.
        frac = 1.0 - self.slow_gain * abs(e) / self.half_width_px
        v = self.target_speed * max(self.v_min_frac, min(1.0, frac))
        return v, yaw

    def tick(self):
        forward, yaw = 0., 0.
        try:
            observation = self.process_image(self.image)
            estimate = self.update_estimate(observation, self.imu)
            action = self.choose_action(estimate)
            forward, yaw = self.compute_motion(action)
            if not all(math.isfinite(v) for v in (forward, yaw)):
                raise ValueError('controller returned a nonfinite command')
            self.previous_error = None
        except Exception as exc:
            forward, yaw = 0., 0.
            error = str(exc)
            if error != self.previous_error:
                self.get_logger().error(f'Navigation failed; requesting zero motion: {error}')
                self.previous_error = error
        msg = MotionRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.forward_speed_m_s, msg.yaw_rate_rad_s = float(forward), float(yaw)
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
