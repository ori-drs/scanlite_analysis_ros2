# Scanlite Analysis ROS2 Package

This package provides tools for real-time ultrasound bone segmentation, 3D reconstruction, ROS integration, and interactive GUI visualization.

## 📁 Project Structure

```
scanlite_analysis_ros2/
├── launch/
│   └── segmentation_launch.py
├── scanlite_analysis_ros2/
│   ├── segmentation_node.py
│   ├── reconstruction_node.py
        ├─► Real-time 3D reconstruction
    │   ├─► Interactive 3D visualization
    │   └─► Point cloud publishing
│   ├── rosbagtest.py
│   ├── LiveDemoTool.py
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

### `reconstruction_node.py`
- Performs real-time 3D reconstruction of segmented bone surfaces
- Subscribes to `/us_image` and `/vicon/clarius_5_marker/clarius_5_marker` topics
- Publishes:
  - Segmented images (`/seg_bone/image`)
  - 3D point cloud data (`/rec_bone/points`)
- Features interactive 3D visualization using Matplotlib
- Includes coordinate axes visualization for scanner position tracking

## 🖥 GUI Tools (Tkinter)

| Tool                  | Description                                             |
|-----------------------|---------------------------------------------------------|
| **DataAcqTool.py**    | GUI tool for acquiring real-time data from ultrasound devices. Built using Python's Tkinter.|
| **LiveDemoTool.py**   | Real-time bone segmentation visualization tool using Python's Tkinter. Displays segmentation overlay dynamically.|


## Functional Overview

### **ROS Nodes**

- **`segmentation_node.py`**
  - Publishes segmentation results independently via ROS topics.

### ROS Bag Testing (CatMausApp)

- **`rosbagtest.py`**
  - Select and play ROS bags for validating segmentation.

#### Features:
- **ROS Integration**:
  - Subscribes to `/us_image` and Vicon motion topics (`/vicon/clarius_5_marker/clarius_5_marker`).
  - Synchronizes motion data with ultrasound images (`bound_img_motion()`).

- **Graphical User Interface (GUI)**:
  - Developed with Tkinter.
  - Displays ultrasound images with segmentation overlays using Matplotlib.
  - Interactive controls for segmentation parameters (F0, F1, Bth, JC).

- **Bone Segmentation**:
  - Real-time segmentation via the `BoneSeg()` method.

- **3D Reconstruction**:
  - Converts segmented 2D data into interactive 3D visuals using Vicon motion tracking data.
  - Features interactive zooming and reset functionalities.

- **User Interaction**:
  - Controls for segmentation start/stop, zoom, and reset functionalities.
  - Live updating visualization.

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

Run the 3D reconstruction node:
```bash
ros2 run scanlite_analysis_ros2 reconstruction_node.py
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

- ROS1 ( Noetic)
- Python 3.x
- Tkinter
- Matplotlib
- NumPy

---
This project relies on **PyCATMAUS**, a library used for segmentation and transformation functions.

🔗 **Repository:** [PyCATMAUS GitHub](https://github.com/ori-drs/cat_and_maus/tree/master/cat_maus_gui/scripts/PyCATMAUS)


