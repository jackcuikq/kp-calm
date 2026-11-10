"""Receive sensors and request motion through the supplied safety node.

The supplied hooks return no action. Implement perception and control here
or distribute those responsibilities across additional team nodes.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from course_robot_interfaces.msg import MotionRequest


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.target_speed = self.declare_parameter('requested_forward_speed_m_s', .05).value
        self.bridge = CvBridge()
        self.image = self.image_stamp = self.imu = None
        self.previous_error = None
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

    def process_image(self, image):
        """Estimate road position from a BGR image; return None if unknown.

        Decide what observation to return and document its units. Check for
        a missing image before using it. You decide how old an image may be
        before it should no longer guide the car; image_stamp is available.
        Agree on units, validity and timestamps for observations passed
        between your team's processing hooks.
        """
        return None

    def update_estimate(self, observation, imu):
        """Combine current observations with state retained between callbacks.

        Filtering is optional. Explain any use of IMU axes and units, and
        distinguish a road-relative estimate from a global position.
        """
        return None

    def choose_action(self, estimate):
        """Choose a desired behavior, such as stop or follow the road."""
        return None

    def compute_motion(self, action):
        """Return (forward m/s, left-turn rad/s); no action means zero motion.

        Implement your controller here. A positive yaw request turns left.
        For student-determined separation b (metres), safety converts to treads
        v - b*w/2 and v + b*w/2. Y selects the global tread-speed limit:
        Low 0.15, Medium 0.60, or High 1.40 m/s. If either exceeds that limit,
        it scales both together, preserving their ratio without disarming.
        Invalid or stale requests still stop and disarm. See the README.
        """
        return 0., 0.

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
