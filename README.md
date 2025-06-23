```markdown
# Simulator

A ROS/Gazebo-based simulator and perception/control stack for an Ackermann‐steering RC car.

## Features

- **Sign Detection**: YOLOv5 + PnP pose estimation, color/contour filtering, and outlier rejection  
- **Custom EKF Fusion**: Fuses wheel odometry, IMU, and sign-based landmark observations  
- **Baseline EKF**: Fuses only wheel odometry + IMU  
- **Sensor Characterization**: Estimate noise covariances from ground-truth  
- **Offline Evaluation Scripts**: Compare trajectories vs. ground truth  
- **ROS Nodes**: For control, visualization, logging, etc.  
- **Gazebo Simulation Environment**: With map and objects  

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation & Build](#installation--build)
3. [Running the Simulation](#running-the-simulation)
4. [Sensor Characterization](#sensor-characterization)
5. [Offline Evaluation](#offline-evaluation)
6. [Launch Files & Node Responsibilities](#launch-files--node-responsibilities)
7. [Repository Structure (Excerpt)](#repository-structure-excerpt)
8. [Contributing](#contributing)
9. [License](#license)
10. [Contact](#contact)

---

## Prerequisites

- **OS**: Ubuntu 20.04 LTS  
- **ROS**: Noetic  
- **Gazebo**: Gazebo 11 (with ROS Noetic integration)  
- **Python**: 3.x (ROS Noetic default)  
- **C++**: C++14 (ROS Noetic toolchain)  
- **CUDA/PyTorch** (optional): For GPU-accelerated YOLOv5 inference  
- **Catkin Workspace**: A ROS catkin workspace (existing or new)  

### System Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-ros-base \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-rosbag \
  ros-noetic-roslaunch \
  ros-noetic-rviz \
  ros-noetic-robot-localization \
  ros-noetic-gazebo-ros \
  python3-opencv \
  python3-pip
pip3 install numpy pandas matplotlib opencv-python
# If using GPU for YOLOv5:
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu113
```

---

## Installation & Build

### Clone into a Catkin Workspace

**Existing workspace:**
```bash
cd ~/catkin_ws/src
git clone https://github.com/mitsosasim/Simulator.git
```

**Or new workspace:**
```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/mitsosasim/Simulator.git
```

### Install ROS Dependencies

Ensure all listed packages above are installed.

### Install Python Dependencies

```bash
pip3 install torch torchvision numpy opencv-python pandas matplotlib
```

### Build

```bash
cd ~/sim_ws
catkin_make   # or: catkin build
source setup.bash
```

---

## Running the Simulation

### Source Workspace

```bash
source ~/sim_ws/devel/setup.bash
```

### Launch Gazebo

```bash
roslaunch sim_pkg map_with_all_objects.launch
```

This command launches:
- YOLO sign detection node
- Custom EKF node
- robot_localization EKF node

**To run nodes separately:**
```bash
rosrun example sign_detector_node.py _model:=/path/to/best.pt _yolo_repo:=/path/to/YOLOv5-traffic
rosrun example ekf_fusion_node
rosrun example robot_localization
```

### Run Control Node

```bash
rosrun example control.py
```
Drive the car via joystick/keyboard.

---

## Sensor Characterization

Estimate sensor noise covariances:

```bash
rosrun example characterize_sensors.py _model_name:=automobile
```

- Subscribes to `/ground_truth_path`, `/automobile/wheel_encoder/odometry`, `/automobile/IMU`.
- Buffers ground-truth poses, interpolates to match odom/IMU timestamps, computes residuals (mean/std), and suggests optimal variances for `R_odom` and `R_imu`.

---

## Offline Evaluation

### Record rosbag

```bash
# Manual stop:
rosbag record /ground_truth_path /ekf/path /robot_localization/path

# Fixed duration (e.g., 60s):
timeout 60s rosbag record /ground_truth_path /ekf/path /robot_localization/path

# or:
rosbag record --duration=60 /ground_truth_path /ekf/path /robot_localization/path
```

### Run Evaluation Script

```bash
rosrun example evaluation.py /path/to/yourbag.bag
```

- Extracts final poses from Path topics.
- Builds DataFrames for ground truth, custom EKF, baseline EKF.
- Aligns by nearest timestamp (tolerance ~0.05 s).
- Computes position/yaw errors; metrics: RMSE, mean, max → `ekf_comparison_metrics.csv`.
- Creates plots.

---

## Launch Files & Node Responsibilities

- **map_with_all_objects.launch**: Main launch file for simulation, perception, and localization.
- **sign_detector_node.py**: YOLOv5-based sign detection and pose estimation.
- **ekf_fusion_node**: Custom EKF fusing odometry, IMU, and sign landmarks.
- **robot_localization**: Baseline EKF (odometry + IMU).
- **control.py**: Manual or autonomous vehicle control.

---

## Repository Structure (Excerpt)

```
Simulator/
├── src/
│   ├── example/
│   │   ├── src/
│   │   │   ├── sign_detector_node.py
│   │   │   ├── ekf_fusion_node.cpp
│   │   │   ├── robot_localization.cpp
│   │   │   ├── control.py
│   │   │   └── ...
│   ├── sim_pkg/
│   │   ├── launch/
│   │   │   └── map_with_all_objects.launch
│   │   └── ...
│   ├── utils_ros/
│   │   ├── rviz.rviz
│   │   └── ...
│   └── ...
├── README.md
└── ...
```

---

## Contributing

Contributions, bug reports, and feature requests are welcome!  
Please open an issue or submit a pull request.

---



## Contact

For questions or collaboration, contact:  
**mitsosasim** (github.com/mitsosasim)

---
