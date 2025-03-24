#!/usr/bin/env python3

import os
import sys
# print(sys.path)
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
current_dir = os.path.dirname(os.path.abspath(__file__))
pycatmaus_path = os.path.join(current_dir, 'PyCATMAUS')

if os.path.exists(pycatmaus_path) and pycatmaus_path not in sys.path:
    sys.path.append(pycatmaus_path)
    print(f"Added PyCATMAUS path: {pycatmaus_path}")

try:
    from PyCATMAUS.SegBone import RunBoneSeg 
    # from PyCATMAUS.TransFunction import quat2rotm, transfrom_i2l, imgp
    print("PyCATMAUS successfully loaded!")
except ModuleNotFoundError as e:
    print(f"PyCATMAUS import failed: {e}")

# Add the directory containing PyCATMAUS to the Python path

class BoneSegmentationNode(Node):
    def __init__(self):
        super().__init__('bone_segmentation_node')
        self.get_logger().info("Starting Bone Segmentation Node")

        # Try to locate PyCATMAUS


        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create a subscriber for the ultrasound image topic
        self.subscription = self.create_subscription(
            Image,
            "/us_image",
            self.image_callback,
            10
        )
        # Publisher for segmented images
        self.seg_image_pub = self.create_publisher(Image, "/seg_bone/image", 10)

        # Declare parameters with default values
        self.declare_parameter("segmentation.size", 2)
        self.declare_parameter("segmentation.color_r", 0)
        self.declare_parameter("segmentation.color_g", 0)
        self.declare_parameter("segmentation.color_b", 255)

        self.get_logger().info("Bone Segmentation Node Started!")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format (grayscale)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

            # Apply Bone Segmentation (using fixed parameters F0, F1, Bth, JC)
            segmented_coords = RunBoneSeg(cv_image, F0=5, F1=3, Bth=0.05, JC=3)

            # Retrieve segmentation parameters
            seg_size = self.get_parameter("segmentation.size").value
            color_r = self.get_parameter("segmentation.color_r").value
            color_g = self.get_parameter("segmentation.color_g").value
            color_b = self.get_parameter("segmentation.color_b").value

            # OpenCV uses BGR color order
            segmentation_color = (color_b, color_g, color_r)

            # Convert grayscale image to BGR to overlay colored segmentation
            seg_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            # Extract segmentation points
            x_coords = segmented_coords[0]
            y_coords = segmented_coords[1]

            # Draw segmentation with adjustable thickness and color
            for i in range(len(x_coords)):
                x = int(x_coords[i])
                y = int(y_coords[i])
                if 0 <= x < seg_image.shape[1] and 0 <= y < seg_image.shape[0]:
                    cv2.circle(seg_image, (x, y), radius=seg_size, color=segmentation_color, thickness=-1)

            # Convert back to ROS Image format and publish
            seg_msg = self.bridge.cv2_to_imgmsg(seg_image, "bgr8")
            self.seg_image_pub.publish(seg_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BoneSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
