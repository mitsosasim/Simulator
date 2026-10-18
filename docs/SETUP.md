# Setup and reproduction

[Overview](../README.md) · [Architecture and results](ARCHITECTURE.md)

## Supported environment and current status

The original workflow targets **Ubuntu 20.04, ROS Noetic, Gazebo 11, Python 3, and a C++ toolchain compatible with Noetic**. CUDA is optional. ROS Noetic and Ubuntu 20.04 are legacy targets, so use an isolated compatible environment when working from a newer OS.

The cloud host used for documentation is Debian 13. An Ubuntu 20.04 / ROS Noetic Docker image runs successfully on it. A ROS publisher/subscriber round-trip and builds of `utils_ros` and `traffic_light_pkg` passed during environment setup. Full Gazebo and perception dependencies remain blocked by the network policy for `snapshots.ros.org`. This does not invalidate the historical experiments, and it does not establish that the full simulation runs in the current cloud instance.

## Verified software-only build

From the repository root, the following builds the repository's messages and traffic light Python wrapper without changing the checkout. It requires Docker access and the official ROS image. The image digest pins the version used in validation.

```bash
sim_ws=$(mktemp -d /tmp/simulator-messages.XXXXXX)
docker run --rm \
  -v "$PWD:/repo:ro" -v "$sim_ws:/ws" \
  ros@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d \
  bash -c '
    set -e
    mkdir -p /ws/src
    ln -s /repo/src/utils_ros /ws/src/utils_ros
    ln -s /repo/src/traffic_light_pkg /ws/src/traffic_light_pkg
    cd /ws
    catkin_make -j2 -l2
    source devel/setup.bash
    python3 -c "from utils_ros.msg import IMU, SignLabelArray; print(IMU._type, SignLabelArray._type)"
  '
```

Expected output includes a completed Catkin build and `utils_ros/IMU utils_ros/SignLabelArray`. The `traffic_light_pkg` CMake file emits an existing warning about generating zero messages. `utils_ros` does generate its three messages. Build output remains in the directory stored in `$sim_ws` on the host.

Cloud tasks are already isolated. Use the existing checkout and do not create a Git worktree for setup.

## Full simulation prerequisites

The following is the repository's full development path, updated to use an external build workspace. It has **not** passed end-to-end validation in this cloud instance. On a configured Ubuntu 20.04 / Noetic machine, install the ROS integration packages:

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

Noetic images may use the final snapshot at `snapshots.ros.org/noetic/final/ubuntu`. Preserve signed package verification and TLS checks. If a managed HTTPS proxy is used, install its trusted CA in the container and provide the supported proxy route. Do not disable verification to work around trust errors. Cloud environment settings need the relevant ROS package domains in addition to the Ubuntu and Python package destinations.

Build outside the source checkout:

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

The generated workspace and environment variables must be available in each ROS terminal. Existing source directories and package identifiers stay in place.

## Perception dependencies and model

Create a Python 3 virtual environment with access to the system ROS packages, then install the vendored YOLOv5 requirements:

```bash
python3 -m venv --system-site-packages /tmp/simulator-python
source /tmp/simulator-python/bin/activate
python3 -m pip install -r "$repo_dir/YOLOv5-traffic/yolov5/requirements.txt"
```

The requirements use lower bounds rather than a lockfile. This command is a reproduction starting point, not a verified frozen training environment. Choose compatible Python, NumPy, OpenCV, PyTorch, and torchvision versions for Noetic and the supplied weights. Do not upgrade the ROS system interpreter in place.

Trained traffic sign weights are not tracked. Use the author's checkpoint, camera calibration, sign dimensions, and matching class labels. The dataset configuration currently contains absolute paths, so use a local copy with corrected paths for training. Training from scratch is outside the documented inference path.

Once the simulator, camera topics, and TF tree are running, explicitly override the detector's machine-specific defaults:

```bash
rosrun example sign_detector_node.py \
  _model:=/absolute/path/to/best.pt \
  _yolo_repo:="$repo_dir/YOLOv5-traffic/yolov5" \
  _device:=cpu
```

The node still requires a display for its OpenCV window. Default detection confidence is `0.6`, and its temporal pose threshold defaults to `0.08 m`. These are source defaults, not measured accuracy or range claims.

## Simulation startup and readiness

The historical entry point is:

```bash
roslaunch sim_pkg map_with_all_objects.launch
```

Before using it on a new machine, account for its hardcoded RViz configuration path and detector defaults through a local launch copy or supported private parameter overrides. Preserve the original launch file for the archived experiment. The main launch starts both local C++ filters, which broadcast the same TF child. Inspect the transform publishers before relying on that tree. Start optional viewers only when a display is available.

Successful startup requires more than a running process. Verify:

```bash
rosservice call /gazebo/get_world_properties '{}'
rostopic echo -n 1 /automobile/wheel_encoder/odometry
rostopic echo -n 1 /automobile/IMU
rostopic echo -n 1 /ground_truth_path
rostopic echo -n 1 /sign_detector/sign_poses
rostopic echo -n 1 /ekf/path
```

Check that the car exists, sensor timestamps advance, detections refer to `chassis::link`, and the EKF path contains poses. The EKF intentionally waits for the ground-truth initial pose. These full-stack readiness checks remain unexecuted in this cloud instance.

## Recording, characterization, and evaluation

On a working simulation, sensor characterization runs separately:

```bash
rosrun example characterize_sensors.py
```

Stop it normally to print residual statistics. Record the three paths for offline comparison:

```bash
rosbag record --duration=60 /ground_truth_path /ekf/path /robot_localization/path
```

Run evaluation in a new output directory to preserve the tracked historical figures:

```bash
mkdir -p /tmp/simulator-evaluation
cd /tmp/simulator-evaluation
MPLBACKEND=Agg rosrun example evaluation.py /absolute/path/to/recording.bag
```

The script writes metrics and plots to its current directory. It aligns samples within `0.05 s` and needs nonempty path messages. No original rosbag is included. Interpret the local comparison filter according to the [experimental evidence notes](ARCHITECTURE.md#experimental-evidence).
