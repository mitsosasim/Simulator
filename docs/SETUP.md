# Setup

[Project](../README.md) · [Architecture and results](ARCHITECTURE.md)

## Environment

Use **Ubuntu 20.04, ROS Noetic, Gazebo 11, and Python 3**. On a newer OS, use a compatible Ubuntu/ROS container or virtual machine. CUDA is optional. Gazebo, RViz, and the detector's OpenCV window require a display.

With ROS Noetic's package repository configured:

```bash
sudo apt-get update
sudo apt-get install -y build-essential libeigen3-dev \
  ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs \
  ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  ros-noetic-nav-msgs ros-noetic-geometry-msgs ros-noetic-sensor-msgs \
  ros-noetic-vision-msgs ros-noetic-map-server ros-noetic-rviz \
  ros-noetic-robot-localization python3-opencv python3-pandas \
  python3-matplotlib python3-pip python3-venv
```

Noetic is a legacy distribution. Images may use its final package snapshot at `snapshots.ros.org/noetic/final/ubuntu`.

## Build

From the repository root, create a separate Catkin workspace:

```bash
repo_dir="$PWD"
sim_ws=$(mktemp -d /tmp/simulator-full.XXXXXX)
mkdir -p "$sim_ws/src"
ln -s "$repo_dir/src/"* "$sim_ws/src/"
source /opt/ros/noetic/setup.bash
cd "$sim_ws"
catkin_make -j2 -l2
source devel/setup.bash
export GAZEBO_MODEL_PATH="$repo_dir/src/models_pkg:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$sim_ws/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
```

Source the workspace and set the Gazebo paths in each ROS terminal.

## Perception

Create a virtual environment with access to system ROS packages:

```bash
python3 -m venv --system-site-packages /tmp/simulator-python
source /tmp/simulator-python/bin/activate
python3 -m pip install -r "$repo_dir/YOLOv5-traffic/yolov5/requirements.txt"
```

The requirements specify minimum versions. Use versions compatible with Noetic's Python 3.8 and the trained checkpoint.

**Trained weights are not included.** Provide a traffic-sign checkpoint with class labels matching the detector and landmark map. Camera calibration and physical sign dimensions must also match the simulation. For training, replace the absolute dataset paths in a local copy of `YOLOv5-traffic/traffic.yaml`.

The detector supports private parameters for its machine-specific paths:

```bash
rosrun example sign_detector_node.py \
  _model:=/absolute/path/to/best.pt \
  _yolo_repo:="$repo_dir/YOLOv5-traffic/yolov5" \
  _device:=cpu
```

Use this command for a separately launched detector. When starting it through a launch file, supply the same parameters there.

## Run the simulation

The main entry point is:

```bash
roslaunch sim_pkg map_with_all_objects.launch
```

Before running, adapt a local launch copy: replace the hardcoded RViz configuration path with `$repo_dir/src/utils_ros/rviz.rviz`, set the detector parameters above, and choose one publisher for the `map → chassis::link` transform. The main launch starts two filters that publish that transform.

To inspect the running system:

```bash
rosservice call /gazebo/get_world_properties '{}'
rostopic echo -n 1 /automobile/wheel_encoder/odometry
rostopic echo -n 1 /automobile/IMU
rostopic echo -n 1 /ground_truth_path
rostopic echo -n 1 /sign_detector/sign_poses
rostopic echo -n 1 /ekf/path
```

The world should contain the car, sensor timestamps should advance, and the EKF path should contain poses. The filter waits for a ground-truth initial pose.

## Record and evaluate

Run sensor characterization separately:

```bash
rosrun example characterize_sensors.py
```

Stop it normally to print residual statistics. Record the reference and estimated paths:

```bash
rosbag record --duration=60 /ground_truth_path /ekf/path /robot_localization/path
```

Evaluate a recording from a separate output directory:

```bash
mkdir -p /tmp/simulator-evaluation
cd /tmp/simulator-evaluation
MPLBACKEND=Agg rosrun example evaluation.py /absolute/path/to/recording.bag
```

The script needs nonempty path messages and writes metrics and plots to its current directory. Original experiment rosbags are not included. See [results and comparison settings](ARCHITECTURE.md#results).
