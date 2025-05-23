#!/usr/bin/env python3

import os
import sys
import time  # At the top of your file

# --- Add this block before importing matplotlib.pyplot ---
if not os.environ.get("DISPLAY"):
    import matplotlib
    matplotlib.use('Agg')
# --------------------------------------------------------

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct

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

class BoneSegmentationNode(Node):
    def __init__(self):
        self.recons_x = []
        self.recons_y = []
        self.recons_z = []
        super().__init__('bone_segmentation_node')
        self.get_logger().info("Starting Bone Segmentation Node")

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.motion_buff = None  # Store motion data

        # Subscribe to ultrasound image topic
        self.subscription = self.create_subscription(
            Image, "/us_image", self.image_callback, 10
        )

        # Subscribe to motion tracking topic
        self.motion_subscription = self.create_subscription(
            TransformStamped, "/rigid_body_transforms", self.motion_callback, 10
        )

        # Publisher for segmented images
        self.seg_image_pub = self.create_publisher(Image, "/seg_bone/image", 10)

        # Add new publisher for point cloud
        self.point_cloud_pub = self.create_publisher(
            PointCloud2, 
            '/rec_bone/points', 
            10
        )

        # Storage for 3D reconstructed points
        self.reconstruction_points = {
            'x': [],
            'y': [],
            'z': []
        }

        # Set up Matplotlib 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Initialize empty plot with fixed axis limits
        self.ax.set_xlim([50, 450])
        self.ax.set_ylim([1150, 1550])
        self.ax.set_zlim([800, 1200])
        
        # Initialize the 3D plot line
        self.reconstruct_3D, = self.ax.plot3D([], [], [], 'b.', ms=1)  # Match LiveDemoTool specs
        
        self.ax.set_xlabel("X Axis")
        self.ax.set_ylabel("Y Axis")
        self.ax.set_zlabel("Z Axis")
        self.ax.set_title("3D Bone Reconstruction")
        plt.draw()
        plt.pause(0.01)

        # Start visualization update loop
        self.timer = self.create_timer(0.05, self.update_plot)  # 0.05 seconds 

        self.get_logger().info("Bone Segmentation Node Started!")

        # Initialize plot timing variables
        self.last_plot_time = None
        self.last_pc_time = None
        self.plot_freq = 0.0

    def image_callback(self, msg):
        """ Processes incoming ultrasound image for bone segmentation. """
        try:
            # Convert ROS Image message to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

            # Apply Bone Segmentation (using fixed parameters)
            segmented_coords = RunBoneSeg(cv_image, F0=1, F1=1, Bth=0.1, JC=1)

            if self.motion_buff is None:
                self.get_logger().info("No motion data available")
            if segmented_coords is None:
                self.get_logger().info("No segmentation coordinates available")

            # If motion data is available, perform 3D transformation
            if self.motion_buff is not None:
                seg_glo = self.transform_to_3d(segmented_coords)
                if seg_glo is not None:
                    # Store reconstructed points
                    self.reconstruction_points['x'].extend(seg_glo[0])
                    self.reconstruction_points['y'].extend(seg_glo[1])
                    self.reconstruction_points['z'].extend(seg_glo[2])

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def motion_callback(self, data):
        """ Stores motion tracking data. """
        self.motion_buff = data.transform

    def transform_to_3d(self, segmented_coords):
        """ Converts 2D segmentation points into 3D using motion tracking data. """
        if self.motion_buff is None or segmented_coords is None:
            return None

        try:
            trans = self.motion_buff.translation
            rot = self.motion_buff.rotation
            pose = np.array([trans.x * 1000, trans.y * 1000, trans.z * 1000])  
            rotm = quat2rotm([rot.w, rot.x, rot.y, rot.z])  # Convert quaternion to rotation matrix

            # Convert 2D segmentation coordinates into 3D (local)
            seg_3D = transfrom_i2l(np.array([segmented_coords[0], segmented_coords[1]]))

            # print(f"Original seg_3D shape: {seg_3D.shape}") 

            # Ensure seg_3D is 3xN (remove extra rows if needed)
            if seg_3D.shape[0] > 3:
                seg_3D = seg_3D[:3, :]  # Keep only the first 3 rows

            # print(f"Fixed seg_3D shape: {seg_3D.shape}")  

            # Apply transformation matrix
            transform_matrix = np.column_stack((rotm, pose))
            transform_matrix = np.row_stack((transform_matrix, [0, 0, 0, 1]))

            # print(f"transform_matrix shape: {transform_matrix.shape}")  # Debugging print
            # Apply transformation matrix to 3D coordinates
            seg_glo = np.matmul(transform_matrix, np.vstack((seg_3D, np.ones((1, seg_3D.shape[1])))))

            # print(f"seg_glo shape: {seg_glo.shape}")  # Debugging print

            return seg_glo[:3]  # Extract X, Y, Z coordinates

        except Exception as e:
            self.get_logger().error(f"Error transforming to 3D: {e}")
            return None

    def create_point_cloud_msg(self, points_x, points_y, points_z):
        """Creates a PointCloud2 message from x, y, z coordinates."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vicon"

        # Convert points to meters
        points_x = np.array(points_x) / 1000.0
        points_y = np.array(points_y) / 1000.0
        points_z = np.array(points_z) / 1000.0

        # Define fields including RGB color
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        msg.point_step = 16  # 4 bytes per float * 3 floats + 4 bytes for rgba
        
        # Create color data (blue with low alpha for smaller appearance)
        # Format: 0xAABBGGRR (alpha, blue, green, red)
        blue_semi_transparent = 0x40FF0000  # Alpha=0x40 (64), Blue=0xFF, Green=0x00, Red=0x00
        
        # Pack points and color into binary data
        points_count = len(points_x)
        points = np.zeros((points_count, 4), dtype=np.float32)
        points[:, 0] = points_x
        points[:, 1] = points_y
        points[:, 2] = points_z
        points[:, 3].view(np.uint32)[:] = blue_semi_transparent
        
        msg.data = points.tobytes()
        msg.height = 1
        msg.width = points_count
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.is_bigendian = False
        
        return msg

    def update_plot(self):
        """Updates both Matplotlib and RViz visualizations."""
        x = self.reconstruction_points['x']
        y = self.reconstruction_points['y']
        z = self.reconstruction_points['z']

        # --- Frequency measurement ---
        now = time.time()
        if self.last_plot_time is not None:
            dt = now - self.last_plot_time
            if dt > 0:
                self.plot_freq = 1.0 / dt
                print(f"Matplotlib plot update frequency: {self.plot_freq:.2f} Hz")
        self.last_plot_time = now
        # ----------------------------

        if len(x) > 0:
            self.ax.cla()
            self.ax.scatter(x, y, z, c='blue', marker='.', s=2)
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")
            self.ax.set_title("3D Bone Reconstruction")
            self.ax.set_xlim([-1000, -300])
            self.ax.set_ylim([-600, 300])
            self.ax.set_zlim([300, 700])
            plt.draw()  # Redraw the plot
            plt.pause(0.01)  # Pause briefly to allow the plot to update

            # Publish point cloud for RViz
            point_cloud_msg = self.create_point_cloud_msg(
                self.reconstruction_points['x'],
                self.reconstruction_points['y'],
                self.reconstruction_points['z']
            )

            # --- PointCloud2 frequency measurement ---
            now_pc = time.time()
            if self.last_pc_time is not None:
                dt_pc = now_pc - self.last_pc_time
                if dt_pc > 0:
                    self.pc_freq = 1.0 / dt_pc
                    print(f"PointCloud2 publish frequency: {self.pc_freq:.2f} Hz")
            self.last_pc_time = now_pc
            # -----------------------------------------

            self.point_cloud_pub.publish(point_cloud_msg)

        # Limit the number of points stored for reconstruction
        max_points = 10000  # or another reasonable limit
        for key in self.reconstruction_points:
            if len(self.reconstruction_points[key]) > max_points:
                self.reconstruction_points[key] = self.reconstruction_points[key][-max_points:]

def main(args=None):
    rclpy.init(args=args)
    node = BoneSegmentationNode()
    
    try:
        plt.ion()  # Enable interactive mode
        # plt.show(block=False)  # Non-blocking display
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')  # Close all Matplotlib windows
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()