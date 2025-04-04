#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import RigidBodies
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Pose, Vector3
import tf2_ros
import tf_transformations

class RigidBodyVisualizer(Node):
    def __init__(self):
        super().__init__('rigid_body_visualizer')

        self.declare_parameter('mocap4r2_system', 'optitrack')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('marker_lifetime', 0.01)
        self.declare_parameter('namespace', 'mocap4r2_rigidbodies')
        self.declare_parameter('default_color', [0.0, 1.0, 0.0, 1.0])  # RGBA

        self.mocap_system = self.get_parameter('mocap4r2_system').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.default_color = self.get_parameter('default_color').get_parameter_value().double_array_value

        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_rb', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.transform_publisher = self.create_publisher(TransformStamped, 'rigid_body_transforms', 10)
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_body_callback,
            10
        )

        self.get_logger().info("RigidBodyVisualizer node started")

    def mocap_to_rviz(self, pose: Pose) -> Pose:
        """Convert mocap coordinate system to RViz."""
        rviz_pose = Pose()

        if self.mocap_system == 'optitrack':
            rviz_pose.position.x = -pose.position.y
            rviz_pose.position.y = pose.position.x
            rviz_pose.position.z = pose.position.z

            rviz_pose.orientation.x = pose.orientation.x
            rviz_pose.orientation.y = -pose.orientation.y
            rviz_pose.orientation.z = pose.orientation.z
            rviz_pose.orientation.w = pose.orientation.w

        else:
            rviz_pose = pose  # No transform applied

        return rviz_pose

    def rigid_body_callback(self, msg: RigidBodies):
        marker_array = MarkerArray()
        transforms = []

        for i, rb in enumerate(msg.rigidbodies):
            # Transform pose
            pose_rviz = self.mocap_to_rviz(rb.pose)

            # Create visualization marker
            marker = Marker()
            marker.header = msg.header
            marker.ns = self.namespace
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose_rviz
            marker.scale = Vector3(x=0.3, y=0.02, z=0.02)
            marker.color.r = self.default_color[0]
            marker.color.g = self.default_color[1]
            marker.color.b = self.default_color[2]
            marker.color.a = self.default_color[3]
            marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
            marker_array.markers.append(marker)

            # Publish TF if enabled
            if self.publish_tf:
                tf_msg = TransformStamped()
                tf_msg.header = msg.header
                tf_msg.child_frame_id = rb.rigid_body_name
                
                # Create Vector3 translation from position
                tf_msg.transform.translation.x = -rb.pose.position.y  # Already transformed for RViz
                tf_msg.transform.translation.y = rb.pose.position.x   # Already transformed for RViz
                tf_msg.transform.translation.z = rb.pose.position.z

                tf_msg.transform.rotation = rb.pose.orientation

                transforms.append(tf_msg)
                self.transform_publisher.publish(tf_msg)
        if self.publish_tf and transforms:
            self.tf_broadcaster.sendTransform(transforms)

        if self.publisher.get_subscription_count() > 0:
            self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RigidBodyVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
