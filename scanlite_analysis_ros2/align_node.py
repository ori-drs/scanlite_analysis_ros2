#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Header

def umeyama_alignment(src_pts, tgt_pts):
    mu_src = np.mean(src_pts, axis=0)
    mu_tgt = np.mean(tgt_pts, axis=0)
    src_centered = src_pts - mu_src
    tgt_centered = tgt_pts - mu_tgt

    S = np.dot(tgt_centered.T, src_centered) / src_pts.shape[0]
    U, _, Vt = np.linalg.svd(S)
    R = np.dot(U, Vt)
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = np.dot(U, Vt)

    t = mu_tgt.T - R @ mu_src.T
    return R, t

class VioViconAligner(Node):
    def __init__(self):
        super().__init__('vio_vicon_aligner')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.align_once)
        self.vio_frame = 'imu'
        self.vicon_frame = 'scanlite_v16.scanlite_v16_2'
        self.fixed_frame_vio = 'global'
        self.fixed_frame_vicon = 'vicon'

        self.timestamps = []
        self.vio_poses = []
        self.vicon_poses = []
        self.aligned = False
        self.pose_threshold = 100  # Configurable threshold for alignment

        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

    def get_translation(self, tf_msg):
        t = tf_msg.transform.translation
        return np.array([t.x, t.y, t.z])

    def align_once(self):
        if len(self.timestamps) > 100:
            return

        now = rclpy.time.Time()
        try:
            tf_vio = self.tf_buffer.lookup_transform(
                self.fixed_frame_vio, self.vio_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            tf_vicon = self.tf_buffer.lookup_transform(
                self.fixed_frame_vicon, self.vicon_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {str(e)}')
            return

        self.timestamps.append(now.nanoseconds)
        self.vio_poses.append(self.get_translation(tf_vio))
        self.vicon_poses.append(self.get_translation(tf_vicon))

        if len(self.vio_poses) >= self.pose_threshold and not self.aligned:
            R, t = umeyama_alignment(np.array(self.vio_poses), np.array(self.vicon_poses))
            
            # Convert rotation matrix to quaternion for logging
            quat = tf_transformations.quaternion_from_matrix(
                np.vstack((np.hstack((R, np.zeros((3, 1)))), [0, 0, 0, 1]))
            )
            
            self.get_logger().info(f'Aligned rotation (quaternion):\n{quat}')
            self.get_logger().info(f'Aligned translation:\n{t}')

            self.publish_static_tf(R, t)
            self.aligned = True

    def publish_static_tf(self, R, t):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = self.fixed_frame_vio
        static_tf.child_frame_id = 'vio_to_vicon_aligned'
        static_tf.transform.translation.x = t[0]
        static_tf.transform.translation.y = t[1]
        static_tf.transform.translation.z = t[2]

        quat = tf_transformations.quaternion_from_matrix(np.vstack((np.hstack((R, np.zeros((3, 1)))), [0, 0, 0, 1])))
        static_tf.transform.rotation.x = quat[0]
        static_tf.transform.rotation.y = quat[1]
        static_tf.transform.rotation.z = quat[2]
        static_tf.transform.rotation.w = quat[3]

        self.static_tf_broadcaster.sendTransform(static_tf)

def main(args=None):
    rclpy.init(args=args)
    node = VioViconAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()