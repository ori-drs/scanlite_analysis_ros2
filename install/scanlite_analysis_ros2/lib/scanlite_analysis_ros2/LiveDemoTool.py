#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from std_msgs.msg import Header
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import time
from copy import copy
from scipy.io import savemat

print("Starting Bone Segmentation Node")

# Check for PyCATMAUS installation
paths_to_try = [
    os.path.dirname(os.path.abspath(__file__)),  # Current script directory
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "PyCATMAUS")
]

for path in paths_to_try:
    if os.path.exists(path) and path not in sys.path:
        sys.path.append(path)
        print(f"Added PyCATMAUS path: {path}")

try:
    from PyCATMAUS.SegBone import RunBoneSeg as BoneSeg
    from PyCATMAUS.TransFunction import quat2rotm, transfrom_i2l, imgp
    print("PyCATMAUS successfully loaded!")
except ModuleNotFoundError as e:
    print(f"PyCATMAUS import failed: {e}")

# ROS2 Node for Data Exchange
class DataExchangeNode(Node):
    def __init__(self):
        super().__init__('bone_segmentation_node')
        self.bridge = CvBridge()
        self.motion_buff = None
        self.img_buff = np.zeros((480, 640), dtype=np.uint8)

        # Create subscriptions for image and motion data.
        self.img_subscription = self.create_subscription(
            Image,
            '/us_image',
            self.update_img,
            10
        )
        self.motion_subscription = self.create_subscription(
            TransformStamped,
            '/vicon/clarius_5_marker/clarius_5_marker',
            self.update_motion,
            10
        )
        self.get_logger().info("DataExchangeNode initialized")

    def update_img(self, data):
        try:
            self.img_buff = self.bridge.imgmsg_to_cv2(data, "mono8")
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def update_motion(self, data):
        self.motion_buff = data.transform

# Main GUI App
class CatMausApp:
    def __init__(self, master):
        self.master = master
        self.master.title("CAT&MAUS Demo")

        # GUI Colors
        self.bg_color = '#d9d9d9'
        self.btn_on = '#229955'
        self.btn_off = '#bbbbbb'

        # Create the matplotlib figure and canvas.
        self.fig = plt.figure(figsize=(14, 6.4), dpi=100, facecolor=self.bg_color, tight_layout=True)
        ax1 = self.fig.add_subplot(121, title="Ultrasound Image", frameon=False)
        ax1.tick_params(axis='both', which='both', bottom=False, left=False, labelleft=False, labelbottom=False)

        self.USI1 = ax1.imshow(np.zeros((480, 640)), cmap='gray', vmin=0, vmax=255)
        self.Seg1, = ax1.plot([], [], 'r.')

        self.ax2 = self.fig.add_subplot(122, projection='3d')
        self.ax2.set_xlim([50, 450])
        self.ax2.set_ylim([1150, 1550])
        self.ax2.set_zlim([800, 1200])
        self.scanner_x, = self.ax2.plot3D([0, 20], [0, 0], [0, 0], color='r')
        self.scanner_y, = self.ax2.plot3D([0, 0], [0, 20], [0, 0], color='g')
        self.scanner_z, = self.ax2.plot3D([0, 0], [0, 0], [0, 20], color='b')
        # Note: The following assumes that reconstruct_3D has custom methods get_data_3d() and set_data_3d().
        self.reconstruct_3D, = self.ax2.plot3D([], [], [], 'b.', ms=1)

        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack()

        # Buttons for connecting data, segmentation, reconstruction, and saving data.
        self.button_connect = tk.Button(master, text="Connect Data", bg=self.btn_off, command=self.connect_data)
        self.button_connect.pack()
        self.button_seg = tk.Button(master, text="BoneSeg", bg=self.btn_off, command=self.toggle_segmentation)
        self.button_seg.pack()
        self.button_reconstruct = tk.Button(master, text="Start Reconstruction", bg=self.btn_off, command=self.toggle_reconstruction)
        self.button_reconstruct.pack()
        self.button_save = tk.Button(master, text="Save Data", bg=self.btn_off, command=self.save_reconstruction)
        self.button_save.pack()

        # Flags to control connection, segmentation, and reconstruction.
        self.data_node = None
        self.var_connected = False
        self.var_segswitch = False
        self.var_reconstruct = False

        # Handle window closing.
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Begin the periodic update loop.
        self.master.after(30, self.update_display)

    def connect_data(self):
        if not self.var_connected:
            self.data_node = DataExchangeNode()
            self.var_connected = True
            self.button_connect.config(text="Connected", bg=self.btn_on)
            print("Connected to DataExchangeNode")
        else:
            self.var_connected = False
            self.button_connect.config(text="Connect Data", bg=self.btn_off)
            print("Disconnected from DataExchangeNode")

    def toggle_segmentation(self):
        self.var_segswitch = not self.var_segswitch
        color = self.btn_on if self.var_segswitch else self.btn_off
        self.button_seg.config(bg=color)
        if not self.var_segswitch:
            self.Seg1.set_xdata([])
            self.Seg1.set_ydata([])

    def toggle_reconstruction(self):
        self.var_reconstruct = not self.var_reconstruct
        text = "Stop Reconstruction" if self.var_reconstruct else "Start Reconstruction"
        color = self.btn_on if self.var_reconstruct else self.btn_off
        self.button_reconstruct.config(text=text, bg=color)

    def update_display(self):
        if self.var_connected:
            # Process incoming ROS2 messages.
            rclpy.spin_once(self.data_node, timeout_sec=0)

            img = self.data_node.img_buff
            self.USI1.set_data(img)

            seg_coords = None
            if self.var_segswitch:
                seg_coords = BoneSeg(img, F0=1, F1=1, Bth=0.1, JC=1)
                self.Seg1.set_xdata(seg_coords[0])
                self.Seg1.set_ydata(seg_coords[1])

            if self.var_reconstruct and self.data_node.motion_buff and seg_coords is not None:
                trans = self.data_node.motion_buff.translation
                rot = self.data_node.motion_buff.rotation
                pose = [trans.x * 1000, trans.y * 1000, trans.z * 1000]
                rotm = quat2rotm([rot.w, rot.x, rot.y, rot.z])
                seg_3D = transfrom_i2l(np.array((seg_coords[0], seg_coords[1])))
                transform_matrix = np.column_stack((rotm, pose))
                transform_matrix = np.row_stack((transform_matrix, [0, 0, 0, 1]))
                seg_glo = np.matmul(transform_matrix, seg_3D)
                # Here we assume that the reconstruct_3D object has get_data_3d() and set_data_3d() methods.
                cloud_3d = self.reconstruct_3D.get_data_3d()
                self.reconstruct_3D.set_data_3d(
                    np.append(cloud_3d[0], seg_glo[0]),
                    np.append(cloud_3d[1], seg_glo[1]),
                    np.append(cloud_3d[2], seg_glo[2])
                )

            self.canvas.draw()

        # Schedule the next update
        self.master.after(30, self.update_display)

    def save_reconstruction(self):
        # Again, this assumes reconstruct_3D implements get_data_3d().
        cloud_3d = self.reconstruct_3D.get_data_3d()
        data = {"x": cloud_3d[0], "y": cloud_3d[1], "z": cloud_3d[2]}
        savemat("reconstruction.mat", data)
        print("3D Reconstruction saved!")

    def on_closing(self):
        if self.var_connected and self.data_node is not None:
            self.data_node.destroy_node()
        self.master.destroy()

if __name__ == "__main__":
    # Initialize ROS2.
    rclpy.init(args=None)

    # Start the Tkinter GUI.
    root = tk.Tk()
    app = CatMausApp(root)
    root.mainloop()

    # Shutdown ROS2 when the GUI is closed.
    rclpy.shutdown()
