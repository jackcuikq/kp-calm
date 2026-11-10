"""Generated messages + real student adapter, using synthetic sensors and no UART."""
import os
import time
if (os.environ.get('ROS_DOMAIN_ID')!='92' or
        os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST' or os.environ.get('CYCLONEDDS_URI')):
    raise SystemExit('Use domain 92, LOCALHOST discovery and unset CYCLONEDDS_URI')
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,Imu
from course_robot_interfaces.msg import MotionRequest
from team_navigation.navigation import Navigation


def main():
    rclpy.init()
    team=Navigation();fixture=Node('starter_smoke_fixture')
    executor=SingleThreadedExecutor();executor.add_node(team);executor.add_node(fixture)
    image=fixture.create_publisher(Image,'camera/image_raw',qos_profile_sensor_data)
    imu=fixture.create_publisher(Imu,'imu/data',qos_profile_sensor_data)
    output=[]
    fixture.create_subscription(MotionRequest,'navigation/request',output.append,qos_profile_sensor_data)
    bridge=CvBridge()
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:
            stamp=fixture.get_clock().now().to_msg()
            frame=bridge.cv2_to_imgmsg(np.zeros((2,2,3),dtype=np.uint8),encoding='bgr8')
            frame.header.stamp=stamp;image.publish(frame)
            inertial=Imu();inertial.header.stamp=stamp;imu.publish(inertial)
            for _ in range(8):executor.spin_once(timeout_sec=.005)
        assert len(output)>10,len(output)
        assert team.image is not None and team.imu is not None
        assert team.image.shape==(2,2,3)
        assert all(m.forward_speed_m_s==m.yaw_rate_rad_s==0. for m in output)
        stamps=[m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in output]
        assert all(b>a for a,b in zip(stamps,stamps[1:]))
        print(f'PASS: starter received BGR/IMU and published {len(output)} fresh zero MotionRequests')
    finally:
        executor.shutdown();fixture.destroy_node();team.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
