#!/usr/bin/env python3

import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Ensure PyCATMAUS is in the path
current_dir = os.path.dirname(os.path.abspath(__file__))
pycatmaus_path = os.path.join(current_dir, 'PyCATMAUS')
if os.path.exists(pycatmaus_path) and pycatmaus_path not in sys.path:
    sys.path.append(pycatmaus_path)

try:
    from PyCATMAUS.SegBone import RunBoneSeg
    from PyCATMAUS.TransFunction import quat2rotm, transfrom_i2l
    print("PyCATMAUS successfully loaded!")
except ModuleNotFoundError as e:
    print(f"PyCATMAUS import failed: {e}")

class BoneReconstructionNode(Node):
    def __init__(self):
        super().__init__('bone_reconstruction_node')
        self.get_logger().info("Starting Bone Reconstruction Node")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to segmented image topic
        self.segmentation_sub = self.create_subscription(
            Image, "/seg_bone/image", self.segmentation_callback, 10
        )

        # Subscribe to motion tracking data
        self.motion_sub = self.create_subscription(
            TransformStamped, "/vicon/clarius_5_marker/clarius_5_marker", self.motion_callback, 10
        )

        # Buffers for segmentation and motion data
        self.seg_coords = None
        self.motion_data = None

        # Matplotlib setup
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X Axis")
        self.ax.set_ylabel("Y Axis")
        self.ax.set_zlabel("Z Axis")
        self.ax.set_title("3D Bone Reconstruction")

        # Start visualization update loop
        self.timer = self.create_timer(0.1, self.update_plot)

    def segmentation_callback(self, msg):
        """ Processes incoming segmented ultrasound image. """
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run segmentation to extract bone points (x, y)
            segmented_coords = RunBoneSeg(cv_image, F0=5, F1=3, Bth=0.05, JC=3)
            self.seg_coords = np.array([segmented_coords[0], segmented_coords[1]])
        
        except Exception as e:
            self.get_logger().error(f"Error processing segmentation data: {e}")

    def motion_callback(self, msg):
        """ Processes incoming motion tracking data. """
        self.motion_data = msg.transform

    def transform_to_3d(self):
        """ Converts 2D segmentation coordinates into 3D using motion tracking data. """
        if self.seg_coords is None or self.motion_data is None:
            return None

        try:
            trans = self.motion_data.translation
            rot = self.motion_data.rotation
            pose = np.array([trans.x * 1000, trans.y * 1000, trans.z * 1000])  # Convert to mm
            rotm = quat2rotm([rot.w, rot.x, rot.y, rot.z])  # Rotation matrix

            # Convert 2D image points to local 3D space
            seg_3D = transfrom_i2l(self.seg_coords)

            # Apply transformation to global coordinate system
            transform_matrix = np.column_stack((rotm, pose))
            transform_matrix = np.row_stack((transform_matrix, [0, 0, 0, 1]))
            seg_glo = np.matmul(transform_matrix, np.vstack((seg_3D, np.ones((1, seg_3D.shape[1])))))

            return seg_glo[:3]  # Extract X, Y, Z coordinates

        except Exception as e:
            self.get_logger().error(f"Error transforming to 3D: {e}")
            return None

    def update_plot(self):
        """ Updates the 3D plot with new reconstructed points. """
        seg_3D = self.transform_to_3d()
        if seg_3D is not None:
            self.ax.clear()
            self.ax.scatter(seg_3D[0], seg_3D[1], seg_3D[2], c='r', marker='o', s=2)
            self.ax.set_xlabel("X Axis")
            self.ax.set_ylabel("Y Axis")
            self.ax.set_zlabel("Z Axis")
            self.ax.set_title("3D Bone Reconstruction")
            plt.draw()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = BoneReconstructionNode()
    
    try:
        plt.ion()  # Interactive mode for live updating
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
