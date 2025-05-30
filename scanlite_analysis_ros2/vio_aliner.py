#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_matrix
import numpy as np

class VIOAligner(Node):
    def __init__(self):
        super().__init__('vio_aligner')

        # Parameters (update with your real alignment)
        self.declare_parameter('parent_frame', 'vicon')
        self.declare_parameter('source_frame', 'imu')
        self.declare_parameter('target_frame', 'imu_aligned')
        self.declare_parameter('T_align_translation', [-0.07843162, 0.863398, 0.50486143])
        self.declare_parameter('T_align_rotation', [-0.072406, -0.289984, -0.055427, 0.952407])
# quaternion: x, y, z, w

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.T_align_trans = np.array(self.get_parameter('T_align_translation').get_parameter_value().double_array_value)
        self.T_align_rot = np.array(self.get_parameter('T_align_rotation').get_parameter_value().double_array_value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30Hz

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            tf_msg = self.tf_buffer.lookup_transform('global', self.source_frame, now)

            # Build 4x4 matrix from source transform
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            T_source = quaternion_matrix([q.x, q.y, q.z, q.w])
            T_source[0:3, 3] = [t.x, t.y, t.z]

            # Alignment transform (from calibration)
            T_align = quaternion_matrix(self.T_align_rot)
            T_align[0:3, 3] = self.T_align_trans

            # Aligned transform
            T_result = np.dot(T_align, T_source)

            # Convert back to TransformStamped
            t_out = TransformStamped()
            t_out.header.stamp = self.get_clock().now().to_msg()
            t_out.header.frame_id = self.parent_frame
            t_out.child_frame_id = self.target_frame
            t_out.transform.translation.x = T_result[0, 3]
            t_out.transform.translation.y = T_result[1, 3]
            t_out.transform.translation.z = T_result[2, 3]

            # Quaternion from rotation matrix
            from tf_transformations import quaternion_from_matrix
            q_out = quaternion_from_matrix(T_result)
            t_out.transform.rotation.x = q_out[0]
            t_out.transform.rotation.y = q_out[1]
            t_out.transform.rotation.z = q_out[2]
            t_out.transform.rotation.w = q_out[3]

            self.tf_broadcaster.sendTransform(t_out)

        except Exception as e:
            self.get_logger().warn(f'Failed to lookup or broadcast transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VIOAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()