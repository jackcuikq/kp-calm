"""Standard RViz markers and TF from telemetry; never publishes motor commands."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class Visualization(Node):
    def __init__(self):
        super().__init__('rvr_visualization')
        self.prefix=self.get_namespace().strip('/') or 'jetson1'
        self.world=f'{self.prefix}/visualization_world';self.body=f'{self.prefix}/base_link'
        self.imu=None;self.imu_at=0.
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id=self.body;t.child_frame_id=f'{self.prefix}/imu_link'
        t.transform.rotation.w=1.
        # Axes-only alignment at the schematic chassis origin, not a surveyed IMU offset.
        self.static.sendTransform(t)
        self.pub=self.create_publisher(MarkerArray,'visualization/markers',1)
        self.create_subscription(Imu,'imu/data',self.on_imu,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)

    def on_imu(self,msg):self.imu=msg;self.imu_at=time.monotonic()

    def tick(self):
        if self.imu is None or time.monotonic()-self.imu_at>1.:return
        stamp=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=self.world;t.child_frame_id=self.body
        t.transform.rotation=self.imu.orientation
        self.tf.sendTransform(t)
        markers=[]
        def marker(kind,pos,scale,color,text=''):
            m=Marker();m.header.stamp=stamp;m.header.frame_id=self.body
            m.ns='rvr';m.id=len(markers);m.type=kind;m.action=Marker.ADD
            m.pose.orientation.w=1.;m.pose.position.x,m.pose.position.y,m.pose.position.z=pos
            m.scale.x,m.scale.y,m.scale.z=scale
            m.color.r,m.color.g,m.color.b,m.color.a=(*color,1.)
            m.text=text;m.lifetime.sec=0;m.lifetime.nanosec=500000000
            markers.append(m)
        marker(Marker.CUBE,(0.,0.,0.),(.30,.22,.09),(.08,.63,.65))
        for y in (.135,-.135):marker(Marker.CUBE,(0.,y,-.015),(.32,.045,.10),(.12,.14,.17))
        marker(Marker.CUBE,(.155,0.,.025),(.025,.17,.045),(1.,.82,.27))
        marker(Marker.CUBE,(-.155,0.,.025),(.025,.17,.045),(.67,.25,.90))
        self.pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init();node=Visualization()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
