#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

class OdomToTransformPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_transform_publisher')

        # Parameters
        self.declare_parameter('marker_lifetime', 0.01)
        self.declare_parameter('namespace', 'odomimu_marker')
        self.declare_parameter('default_color', [0.0, 0.0, 1.0, 1.0])  # RGBA blue

        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.default_color = self.get_parameter('default_color').get_parameter_value().double_array_value

        # Subscriber to Odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odomimu',
            self.odom_callback,
            10
        )

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publisher for TransformStamped
        self.transform_pub = self.create_publisher(
            TransformStamped,
            '/odomimu_transform',
            10
        )

        # Publisher for Marker visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'visualization_marker_odom',
            10
        )

        self.get_logger().info("Publishing TransformStamped and markers from /odomimu")

    def odom_callback(self, msg: Odometry):
        # Create transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        # Publish as TF
        self.tf_broadcaster.sendTransform(t)

        # Publish as message
        self.transform_pub.publish(t)

        # Create and publish marker
        if self.marker_pub.get_subscription_count() > 0:
            marker_array = MarkerArray()
            
            # Create visualization marker
            marker = Marker()
            marker.header = msg.header
            marker.ns = self.namespace
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = msg.pose.pose
            marker.scale = Vector3(x=0.3, y=0.02, z=0.02)
            marker.color.r = self.default_color[0]
            marker.color.g = self.default_color[1]
            marker.color.b = self.default_color[2]
            marker.color.a = self.default_color[3]
            marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
            
            marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
