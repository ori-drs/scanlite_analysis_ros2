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
- Performs real-time 3D reconstruction of segmented bone surfaces
- Supports both IMU and Vicon motion tracking systems
- Subscribes to:
  - `/us_image` for ultrasound images
  - `/odomimu_transform` or `/rigid_body_transforms` for motion tracking
- Publishes:
  - Segmented images (`/seg_bone/image`)
  - 3D point cloud data (`/rec_bone/points`)
- Features interactive 3D visualization using Matplotlib
- Includes coordinate axes visualization for scanner position tracking

### `transformation.py`
- Converts motion capture data to ROS transforms and visualization markers
- Features:
  - Coordinate system conversion for RViz visualization
  - TF broadcasting for rigid body transforms
  - Visualization markers for tracking visualization
  - Configurable parameters for different mocap systems
- Publishes:
  - `/visualization_marker_rb` for RViz visualization
  - `/rigid_body_transforms` for transform data
  - TF frames for each rigid body

### `transformation_odom.py`
- Converts Odometry data to ROS transforms and visualization markers
- Features:
  - Real-time conversion of odometry to transform messages
  - Visualization markers for tracking visualization
  - TF broadcasting for odometry transforms
- Publishes:
  - `/visualization_marker_odom` for RViz visualization
  - `/odomimu_transform` for transform data
  - TF frames for odometry tracking

## Launch Files

To run the system, you can use the following launch commands:

### For IMU-based tracking:
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
## 🖥 GUI Tools (Tkinter)

| Tool                  | Description                                             |
|-----------------------|---------------------------------------------------------|
| **DataAcqTool.py**    | Advanced GUI tool for real-time data acquisition and playback. Features include:<br>- Real-time data recording<br>- Frame-by-frame playback<br>- Data export in multiple formats (PNG, MAT)<br>- Segmentation parameter tuning |
| **LiveDemoTool.py**   | Enhanced real-time bone segmentation visualization tool with:<br>- Interactive 3D reconstruction<br>- Real-time scanner tracking visualization<br>- Data recording and export capabilities<br>- Segmentation parameter controls |

## Functional Overview

### **ROS Nodes**

- **`segmentation_node.py`**
  - Publishes segmentation results independently via ROS topics.

### ROS Bag Testing (CatMausApp)

- **`rosbagtest.py`**
  - Select and play ROS bags for validating segmentation.

#### Features:
- **ROS Integration**:
  - Subscribes to `/us_image` and motion tracking topics
  - Supports both IMU (`/odomimu_transform`) and Vicon (`/rigid_body_transforms`) tracking
  - Synchronizes motion data with ultrasound images

- **Graphical User Interface (GUI)**:
  - Developed with Tkinter
  - Displays ultrasound images with segmentation overlays using Matplotlib
  - Interactive controls for segmentation parameters (F0, F1, Bth, JC)
  - Real-time 3D visualization with coordinate axes

- **Bone Segmentation**:
  - Real-time segmentation via the `BoneSeg()` method
  - Parameter tuning for optimal results

- **3D Reconstruction**:
  - Converts segmented 2D data into interactive 3D visuals
  - Supports multiple motion tracking systems
  - Features interactive zooming and reset functionalities
  - Real-time point cloud publishing for RViz visualization

- **Data Management**:
  - Real-time data recording
  - Frame-by-frame playback
  - Multiple export formats (PNG, MAT)
  - Data visualization and analysis tools

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
# For IMU tracking
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


