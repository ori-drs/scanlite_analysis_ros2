# Scanlite Analysis ROS2 Package

This package provides tools for real-time ultrasound bone segmentation, 3D reconstruction, ROS integration, and interactive GUI visualization.

## 📁 Project Structure

```
scanlite_analysis_ros2/
├── launch/
│   ├── odom_launch.py
│   ├── vicon_launch.py
│   └── segmentation.launch.xml
├── scanlite_analysis_ros2/
│   ├── segmentation_node.py
│   ├── reconstruction_node.py
│   ├── reconstruction_node_vicon.py
│   ├── transformation.py
│   ├── transformation_odom.py
│   ├── rosbagtest.py
│   ├── LiveDemoTool.py
│   ├── DataAcqTool.py
│   └── PyCATMAUS/
│       ├── SegBone.py
│       └── TransFunction.py
├── package.xml
└── CMakeLists.txt
```

## ROS Nodes

### `segmentation_node.py`
- Publishes real-time segmentation results as ROS topics.
- Independent from GUI, facilitating integration into broader ROS systems.

### `reconstruction_node.py` and `reconstruction_node_vicon.py`
- Subscribes to:
  - `/us_image` for ultrasound images
  - `/odomimu_transform` or `/rigid_body_transforms` for motion tracking
- Publishes:
  - Segmented images (`/seg_bone/image`)
  - 3D point cloud data (`/rec_bone/points`)
- Features interactive 3D visualization using Matplotlib
- Includes coordinate axes visualization for scanner position tracking

### `transformation.py`
- Publishes:
  - `/visualization_marker_rb` for RViz visualization
  - `/rigid_body_transforms` for transform data
  - TF frames for each rigid body

### `transformation_odom.py`
- Publishes:
  - `/visualization_marker_odom` for RViz visualization
  - `/odomimu_transform` for transform data
  - TF frames for odometry tracking

## Launch Files

To run the system, you can use the following launch commands:

### For Vio-based tracking:
```bash
ros2 launch scanlite_analysis_ros2 odom_launch.py
```
This launches:
- `transformation_odom.py` for IMU data processing
- `reconstruction_node.py` for 3D reconstruction

### For Vicon-based tracking:
```bash
ros2 launch scanlite_analysis_ros2 vicon_launch.py
```
This launches:
- `transformation.py` for Vicon data processing
- `reconstruction_node_vicon.py` for 3D reconstruction
## Explanation of Segmentation Parameters

| Parameter | Meaning                     | Effect on Segmentation                           |
|-----------|-----------------------------|---------------------------------------------------|
| **F0**    | Energy continuity weight    | Controls smoothness along bone surfaces. Higher values enforce smoother edges. |
| **F1**    | Energy smoothness weight    | Penalizes abrupt depth changes. Higher values produce gradual transitions. |
| **Bth**   | Bone threshold              | Pixel intensity threshold. Lower values include more pixels as bone. |
| **JC**    | Jump constraint             | Limits allowed segmentation path jumps. Higher values increase path flexibility. |

## Usage

Launch the ROS segmentation nodes and related topics:
```bash
ros2 launch scanlite_analysis_ros2 segmentation_launch.py 
```

Run the 3D reconstruction node (choose based on tracking system):
```bash
# For Vio tracking
ros2 run scanlite_analysis_ros2 reconstruction_node.py

# For Vicon tracking
ros2 run scanlite_analysis_ros2 reconstruction_node_vicon.py
```

Start live device acquisition GUI:
```bash
ros2 run scanlite_analysis_ros2 segmentation_node.py
```

Real-time segmentation visualization tool:
```bash
ros2 run scanlite_analysis_ros2 LiveDemoTool.py
```

## Requirements

- ROS2 Humble
- Python 3.x
- Tkinter
- Matplotlib
- NumPy
- OpenCV
- SciPy

---
This project relies on **PyCATMAUS**, a library used for segmentation and transformation functions.

🔗 **Repository:** [PyCATMAUS GitHub](https://github.com/ori-drs/cat_and_maus/tree/master/cat_maus_gui/scripts/PyCATMAUS)

