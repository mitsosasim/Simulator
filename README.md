# Simulator

A ROS/Gazebo-based simulator and perception/control stack for an Ackermann‐steering RC car.  
Key capabilities:
- **Sign detection**: YOLOv5 + PnP pose estimation + color/contour filtering + outlier rejection  
- **Custom EKF fusion**: fuses wheel odometry, IMU, and sign-based landmark observations  
- **Baseline EKF**: fuses only wheel odometry + IMU  
- **Sensor characterization**: estimate noise covariances from ground-truth  
- **Offline evaluation scripts**: compare trajectories vs. ground truth  
- ROS nodes for control, visualization, logging, etc.  
- Gazebo simulation environment with map and objects  

---

## Table of Contents

1. [Prerequisites](#prerequisites)  
2. [Installation & Build](#installation--build)  
3. [Running the Simulation](#running-the-simulation)  
4. [Sensor Characterization](#sensor-characterization)  
5. [Offline Evaluation](#offline-evaluation)  
6. [Launch Files & Node Responsibilities](#launch-files--node-responsibilities)  
   - [`map_with_all_objects.launch`](#map_with_all_objectslaunch)  
7. [Repository Structure (excerpt)](#repository-structure-excerpt)  
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
- **CUDA/PyTorch** (optional): for GPU-accelerated YOLOv5 inference  
- **Catkin workspace**: a ROS catkin workspace (existing or new)  

System dependencies (example installation):
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
