#!/usr/bin/env python3
import os
import rclpy
import subprocess
import tkinter as tk
from tkinter import filedialog
from rclpy.node import Node
from std_msgs.msg import String  # Change this to your actual topic type

class RosbagPlayerNode(Node):
    def __init__(self):
        super().__init__('rosbag_player_node')
        self.get_logger().info("ROS2 Bag Player Node Started")

        # Create a publisher (if needed)
        self.publisher_ = self.create_publisher(String, 'rosbag_status', 10)

    def select_rosbag(self):
        """Opens a file dialog for selecting a ROS 2 bag file."""
        root = tk.Tk()
        root.withdraw()  # Hide the root window
        file_path = filedialog.askopenfilename(
            title="Select a ROS 2 bag file",
            filetypes=[("ROS 2 Bag Files", "*.db3")],  # ROS 2 bags use SQLite3 (.db3)
            initialdir=os.path.expanduser("~")  # Start in user's home directory
        )
        
        if file_path:
            self.get_logger().info(f"Selected ROS 2 bag: {file_path}")
            return file_path
        else:
            self.get_logger().warn("No file selected.")
            return None

    def play_rosbag(self, bag_file):
        """Plays the selected ROS 2 bag file in a separate process."""
        if bag_file:
            self.get_logger().info(f"Playing ROS 2 bag: {bag_file}")
            play_process = subprocess.Popen(["ros2", "bag", "play", bag_file],
                                            stdout=subprocess.DEVNULL, 
                                            stderr=subprocess.DEVNULL)
            return play_process


def main(args=None):
    rclpy.init(args=args)
    node = RosbagPlayerNode()
    try:
        # Select the first ROS 2 bag
        selected_bag1 = node.select_rosbag()
        # Select the second ROS 2 bag
        selected_bag2 = node.select_rosbag()

        # Play both bags if selected
        if selected_bag1 and selected_bag2:
            process1 = node.play_rosbag(selected_bag1)
            process2 = node.play_rosbag(selected_bag2)
            
            # Wait for both processes to finish
            process1.wait()
            process2.wait()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
