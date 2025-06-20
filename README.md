# Autonomous RC Car Simulator

A ROS/Gazebo-based simulator with perception and control stack for an Ackermann-steering RC car.

## Key Capabilities
- **Sign Detection**: YOLOv5 + PnP pose estimation + color/contour filtering + outlier rejection
- **Sensor Fusion**: Custom EKF fusing wheel odometry, IMU, and sign landmarks
- **Localization**: Baseline EKF (wheel odometry + IMU only)
- **Sensor Characterization**: Noise covariance estimation from ground-truth
- **Performance Evaluation**: Offline trajectory comparison scripts
- **Full Simulation**: ROS nodes for control, visualization + Gazebo environment

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Running Simulation](#running-simulation)
4. [Sensor Characterization](#sensor-characterization)
5. [Offline Evaluation](#offline-evaluation)
6. [Launch Files](#launch-files)
7. [Repository Structure](#repository-structure)
8. [Contributing](#contributing)
9. [License](#license)
10. [Contact](#contact)

<a name="prerequisites"></a>
## 1. Prerequisites
- **OS**: Ubuntu 20.04 LTS
- **ROS**: Noetic
- **Gazebo**: 11 (with ROS integration)
- **Python**: 3.x
- **C++**: C++14
- **GPU Acceleration** (Optional): CUDA for YOLOv5 inference

```bash
# Install system dependencies
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

# Install Python packages
pip3 install numpy pandas matplotlib opencv-python

# For GPU acceleration (optional)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu113
<a name="installation"></a>

2. Installation & Build
bash
# Create workspace (if new)
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src

# Clone repository
git clone https://github.com/mitsosasim/Simulator.git

# Install dependencies
cd ~/sim_ws
rosdep install --from-paths src --ignore-src -y

# Install Python dependencies
pip3 install -r src/Simulator/requirements.txt

# Build package
catkin_make
source devel/setup.bash
<a name="running-simulation"></a>

3. Running the Simulation
Standard Launch (All components)
bash
roslaunch sim_pkg map_with_all_objects.launch
Manual Launch (Individual components)
bash
# Start YOLOv5 sign detector
rosrun example sign_detector_node.py _model:=/path/to/best.pt _yolo_repo:=/path/to/YOLOv5-traffic

# Start custom EKF fusion
rosrun example ekf_fusion_node

# Start baseline localization
rosrun example robot_localization

# Start control node
rosrun example control.py
<a name="sensor-characterization"></a>

4. Sensor Characterization
Estimate sensor noise covariances:

bash
rosrun example characterize_sensors.py _model_name:=automobile
Subscribes to /ground_truth_path, /automobile/wheel_encoder/odometry, /automobile/IMU

Buffers ground-truth poses and sensor data

Computes residuals and suggests optimal variances for R_odom/R_imu

<a name="offline-evaluation"></a>

5. Offline Evaluation
1. Record test session
bash
# Record 60-second sample
rosbag record --duration=60 -O test.bag \
  /ground_truth_path \
  /ekf/path \
  /robot_localization/path
2. Evaluate performance
bash
rosrun example evaluation.py test.bag
Outputs:

ekf_comparison_metrics.csv with RMSE, mean, and max errors

Trajectory comparison plots (PDF/PNG)

Pose error over time plots

<a name="launch-files"></a>

6. Launch Files
map_with_all_objects.launch
Launches complete system:

Gazebo simulation world

RC car model

Sign detection node

Custom EKF node

Baseline EKF node

Visualization (RViz)

<a name="repository-structure"></a>

7. Repository Structure
text
Simulator/
├── config/               # Configuration files
├── docs/                # Documentation assets
├── launch/              # Launch files
│   └── map_with_all_objects.launch
├── models/              # 3D models for Gazebo
├── scripts/             # Python nodes
│   ├── characterize_sensors.py
│   ├── control.py
│   ├── evaluation.py
│   └── sign_detector_node.py
├── src/                 # C++ nodes
│   ├── ekf_fusion_node.cpp
│   └── robot_localization.cpp
├── worlds/              # Gazebo simulation worlds
├── CMakeLists.txt
├── package.xml
└── requirements.txt
<a name="contributing"></a>

8. Contributing
Contributions welcome! Please follow:

Fork the repository

Create feature branch (git checkout -b feature/your-feature)

Commit changes (git commit -am 'Add feature')

Push branch (git push origin feature/your-feature)

Open Pull Request

<a name="license"></a>

9. License
Distributed under the MIT License. See LICENSE for details.

<a name="contact"></a>

10. Contact
Project Maintainer: [Your Name]
Email: your.email@example.com
Project Link: https://github.com/mitsosasim/Simulator
