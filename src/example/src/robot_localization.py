#!/usr/bin/env python3
"""
robot_localization_node.py

A single ROS‐Python script that:

  1. Converts your custom sensors into standard topics Robot Localization expects:
     • /automobile/localisation → /gps/fix (geometry_msgs/PoseWithCovarianceStamped)
     • /automobile/IMU        → /imu/data   (sensor_msgs/Imu)
     • /automobile/wheel_encoder/odometry remains nav_msgs/Odometry (no conversion)

  2. Launches Robot Localization’s ekf_localization_node in‐process using roslaunch.scriptapi,
     with parameters that fuse (/automobile/wheel_encoder/odometry, /imu/data, /gps/fix).

  3. Subscribes to /odometry/filtered (EKF output) and republishes a nav_msgs/Path on
     /robot_localization/path for RViz visualization, starting from the true initial pose,
     and keeping the path on the same ground‐plane height.

Usage:
  chmod +x robot_localization_node.py
  rosrun example robot_localization_node.py
"""

import rospy
import roslaunch
import tf2_ros
from tf.transformations import quaternion_from_euler
from utils_ros.msg import localisation, IMU as IMUMsg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path

# ──────────────────────────────────────────────────────────────────────────────
#  Global variables to hold the very first ground‐truth PoseStamped and its z‐height
# ──────────────────────────────────────────────────────────────────────────────

initial_pose_set     = False
initial_pose_stamped = None   # Will hold the first PoseStamped from /ground_truth_path
ground_z             = 0.0    # We will force all future filtered points to this same z

# ──────────────────────────────────────────────────────────────────────────────
def ground_truth_cb(msg: Path):
    """
    Callback to set the initial pose for robot_localization EKF from ground truth.
    Only uses the first pose in the path, and only sets it once at startup.
    """
    global initial_pose_set, initial_pose_stamped, ground_z

    if initial_pose_set or not msg.poses:
        return

    # Grab the very first PoseStamped in /ground_truth_path
    p0 = msg.poses[0]
    initial_pose_stamped = PoseStamped()
    initial_pose_stamped.header = p0.header
    initial_pose_stamped.pose = p0.pose

    # Save its z‐value so the filtered path stays on that ground plane
    ground_z = p0.pose.position.z

    # Publish on /initialpose so the EKF resets exactly here
    init_pose = PoseWithCovarianceStamped()
    init_pose.header = p0.header
    init_pose.pose.pose = p0.pose

    # We trust x,y tightly; give large uncertainty to everything else
    cov = [
        0.01, 0,    0,    0,    0,    0,
         0,   0.01,  0,    0,    0,    0,
         0,    0,    1e6,  0,    0,    0,
         0,    0,    0,    1e6,  0,    0,
         0,    0,    0,    0,    1e6,  0,
         0,    0,    0,    0,    0,    1e6
    ]
    init_pose.pose.covariance = cov

    initialpose_pub.publish(init_pose)
    rospy.loginfo("[robot_localization_node] Initial pose set from ground truth: "
                  "x=%.3f y=%.3f z=%.3f",
                  p0.pose.position.x, p0.pose.position.y, p0.pose.position.z)

    initial_pose_set = True

# ──────────────────────────────────────────────────────────────────────────────
def localisation_cb(msg: localisation):
    """
    Convert custom Localization → /gps/fix (PoseWithCovarianceStamped)
    posA→x, posB→y, rotA→yaw
    """
    gps = PoseWithCovarianceStamped()
    gps.header.stamp = rospy.Time(msg.timestamp, 0)
    gps.header.frame_id = "map"
    gps.pose.pose.position.x = msg.posA
    gps.pose.pose.position.y = msg.posB
    gps.pose.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, msg.rotA)
    gps.pose.pose.orientation.x = q[0]
    gps.pose.pose.orientation.y = q[1]
    gps.pose.pose.orientation.z = q[2]
    gps.pose.pose.orientation.w = q[3]
    # Covariance: trust GPS to ~0.1 m in x,y; very uncertain elsewhere
    cov = [
        0.01, 0,    0,    0,    0,    0,
         0,   0.01,  0,    0,    0,    0,
         0,    0,    1e6,  0,    0,    0,
         0,    0,    0,    1e6,  0,    0,
         0,    0,    0,    0,    1e6,  0,
         0,    0,    0,    0,    0,    1e6
    ]
    gps.pose.covariance = cov
    gps_pub.publish(gps)

# ──────────────────────────────────────────────────────────────────────────────
def imu_cb(msg: IMUMsg):
    """
    Convert custom IMU → /imu/data (sensor_msgs/Imu)
    roll, pitch, yaw → quaternion
    """
    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = "chassis::link"
    q = quaternion_from_euler(msg.roll, msg.pitch, msg.yaw)
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    # Moderate covariance for yaw (but we'll disable yaw in the EKF)
    imu.orientation_covariance = [1e-3, 0,    0,
                                  0,    1e-3, 0,
                                  0,    0,    1e-3]
    # We don’t have real accel/gyro → high covariance
    imu.angular_velocity_covariance = [1e6, 0, 0,
                                       0, 1e6, 0,
                                       0, 0, 1e6]
    imu.linear_acceleration_covariance = [1e6, 0, 0,
                                          0, 1e6, 0,
                                          0, 0, 1e6]
    imu_pub.publish(imu)

# ──────────────────────────────────────────────────────────────────────────────
class PathPublisher:
    """
    Subscribe to /odometry/filtered → publish /robot_localization/path,
    but keep every point on the 'ground_z' plane so RViz stays planar.
    """

    def __init__(self):
        self.path = Path()
        self.path.header.frame_id = "odom"  # still publish in odom frame

        # Flags
        self.have_initial_pose   = False
        self.have_first_filtered = False

        # We will fill these three once ground truth arrives:
        self.init_x = None
        self.init_y = None
        self.init_z = ground_z

        # We will fill these once we see the first non-zero filtered:
        self.first_filt_x = 0.0
        self.first_filt_y = 0.0

        # Subscribe to ground truth (only to set initial pose once)
        rospy.Subscriber("/ground_truth_path", Path, self.ground_truth_cb, queue_size=1)

        # Subscribe to filtered odom (we will wait until have_initial_pose before using)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb, queue_size=10)

        self.pub = rospy.Publisher("/robot_localization/path", Path, queue_size=1)

    def ground_truth_cb(self, msg: Path):
        # Only need the very first pose of ground truth:
        if self.have_initial_pose or not msg.poses:
            return

        first_pose = msg.poses[0]
        self.init_x = first_pose.pose.position.x
        self.init_y = first_pose.pose.position.y
        # ground_z already set globally

        self.have_initial_pose = True
        rospy.loginfo("[PathPublisher] Ground‐truth start set to (%.3f, %.3f, %.3f)",
                      self.init_x, self.init_y, ground_z)

    def odom_cb(self, msg: Odometry):
        # 1) If we haven't yet stored a ground‐truth start, ignore all filtered data:
        if not self.have_initial_pose:
            return

        # 2) Extract the filtered pose from /odometry/filtered:
        x_f = msg.pose.pose.position.x
        y_f = msg.pose.pose.position.y

        # 3) Wait for a “real” filtered reading—i.e. skip (0,0) if that still happens early:
        if (not self.have_first_filtered) and abs(x_f) < 1e-3 and abs(y_f) < 1e-3:
            # still zero/zero; drop it
            return

        # 4) If this is our very first non‐zero filtered reading, lock it in:
        if not self.have_first_filtered:
            self.first_filt_x = x_f
            self.first_filt_y = y_f
            self.have_first_filtered = True
            rospy.loginfo("[PathPublisher] First nonzero filtered = (%.3f, %.3f)", x_f, y_f)

            # Immediately publish a “seed” PoseStamped at the exact ground‐truth start:
            seed = PoseStamped()
            seed.header.stamp = msg.header.stamp
            seed.header.frame_id = "odom"
            seed.pose.position.x = self.init_x
            seed.pose.position.y = self.init_y
            seed.pose.position.z = ground_z
            seed.pose.orientation = msg.pose.pose.orientation
            self.path.poses.append(seed)
            self.path.header.stamp = msg.header.stamp
            self.pub.publish(self.path)

        # 5) Now compute the translated point:
        dx = x_f - self.first_filt_x
        dy = y_f - self.first_filt_y

        mapped_x = self.init_x + dx
        mapped_y = self.init_y + dy
        mapped_z = ground_z  # keep Z constant at ground height

        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = "odom"
        ps.pose.position.x = mapped_x
        ps.pose.position.y = mapped_y
        ps.pose.position.z = mapped_z
        ps.pose.orientation = msg.pose.pose.orientation

        self.path.poses.append(ps)
        self.path.header.stamp = msg.header.stamp
        self.pub.publish(self.path)

# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    rospy.init_node("robot_localization_node", anonymous=True)

    # ──────────────────────────────────────────────────────────────────────────
    #  Option 2 (fallback): Publish a one‐time static transform map→odom
    #  (so RViz can transform “odom” back into “map” even if EKF hasn’t published it yet)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    st = TransformStamped()
    st.header.stamp = rospy.Time.now()
    st.header.frame_id = "map"
    st.child_frame_id  = "odom"
    # Identity transform
    st.transform.translation.x = 0.0
    st.transform.translation.y = 0.0
    st.transform.translation.z = 0.0
    st.transform.rotation.x = 0.0
    st.transform.rotation.y = 0.0
    st.transform.rotation.z = 0.0
    st.transform.rotation.w = 1.0
    static_broadcaster.sendTransform(st)

    # ──────────────────────────────────────────────────────────────────────────
    #  Publishers for converted topics
    gps_pub = rospy.Publisher("/gps/fix", PoseWithCovarianceStamped, queue_size=5)
    imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=5)

    # Publisher for initial pose (so that EKF resets there)
    initialpose_pub = rospy.Publisher(
        "/initialpose",
        PoseWithCovarianceStamped,
        queue_size=1,
        latch=True
    )

    # 1) Subscribe to raw sensors and ground‐truth path
    rospy.Subscriber("/automobile/localisation", localisation, localisation_cb, queue_size=5)
    rospy.Subscriber("/automobile/IMU", IMUMsg, imu_cb, queue_size=5)
    rospy.Subscriber("/ground_truth_path", Path, ground_truth_cb, queue_size=1)
    # EKF will read /automobile/wheel_encoder/odometry directly

    # 2) Push EKF parameters onto the ROS parameter server
    ekf_params = {
        "ekf_filter_node/odom0": "/automobile/wheel_encoder/odometry",
        # 15‐entry: [x,y,z,roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az]
        "ekf_filter_node/odom0_config": [
            False, False, False,   # x,y,z – all false because we only integrate from vx, vyaw
            False, False, False,   # roll, pitch, yaw‐pose unused
            True,  False, False,   # vx used
            False, False, True,    # vyaw used
            False, False, False    # ax, ay, az unused
        ],
        "ekf_filter_node/odom0_queue_size": 10,

        # Disable IMU yaw entirely (because it was always zero in simulation)
        # NOTE: we still feed the IMU.message but we do NOT trust its yaw.
        "ekf_filter_node/imu0": "/imu/data",
        "ekf_filter_node/imu0_config": [
            False, False, False,  # x,y,z‐pose
            False, False, False,  # roll, pitch, yaw – all false!
            False, False, False,  # vx,vy,vz
            False, False, False,  # vroll, vpitch, vyaw
            False, False, False   # ax, ay, az
        ],
        "ekf_filter_node/imu0_queue_size": 10,

        # GPS fuse only x,y
        "ekf_filter_node/gps0": "/gps/fix",
        "ekf_filter_node/gps0_config": [
            True,  True,  False,  # x,y used, z unused
            False, False, False,  # roll, pitch, yaw unused
            False, False, False,  # vx, vy, vz unused
            False, False, False,  # vroll, vpitch, vyaw unused
            False, False, False   # ax, ay, az unused
        ],
        "ekf_filter_node/gps0_queue_size": 10,
        "ekf_filter_node/gps0_differential": False,
        "ekf_filter_node/gps0_relative": False,

        # Frame setup—use chassis::link instead of base_link
        "ekf_filter_node/base_link_frame": "chassis::link",
        "ekf_filter_node/odom_frame": "odom",
        "ekf_filter_node/map_frame": "map",

        # Let the EKF publish TF map→odom → odom→chassis::link
        "ekf_filter_node/publish_tf": True,

        # Tell EKF to honor /initialpose
        "ekf_filter_node/reset_on_pose": True,

        "ekf_filter_node/frequency": 30.0,
        "ekf_filter_node/sensor_timeout": 0.1,
        "ekf_filter_node/stamped_control": False,

        # Process noise (6×6 block for [x,y,z,roll,pitch,yaw])
        "ekf_filter_node/process_noise_covariance": [
            1e-4, 0,    0,    0,    0,    0,
            0,    1e-4, 0,    0,    0,    0,
            0,    0,    1e-6, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ] + [0]*30,  # pad out to 36 total

        # Odometry measurement noise (6×6: [x,y,z,roll,pitch,yaw])
        "ekf_filter_node/odom_noise_covariance": [
            1e-3, 0,    0,    0,    0,    0,
            0,    1e-3, 0,    0,    0,    0,
            0,    0,    1e-6, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ],

        # IMU measurement noise (6×6: [x,y,z,roll,pitch,yaw], but we aren’t using yaw)
        "ekf_filter_node/imu_noise_covariance": [
            1e-6, 0,    0,    0,    0,    0,
            0,    1e-6, 0,    0,    0,    0,
            0,    0,    1e-4, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ],

        # GPS measurement noise (6×6: [x,y,z,roll,pitch,yaw], but we only use x,y)
        "ekf_filter_node/gps_noise_covariance": [
            1e-2, 0,    0,    0,    0,    0,
            0,    1e-2, 0,    0,    0,    0,
            0,    0,    1e-6, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ],
    }

    # Push all EKF parameters onto the parameter server
    for param, value in ekf_params.items():
        rospy.set_param(param, value)

    # 3) Launch ekf_localization_node via roslaunch.scriptapi
    rl = roslaunch.scriptapi.ROSLaunch()
    rl.start()

    ekf_node = roslaunch.core.Node(
        package="robot_localization",
        node_type="ekf_localization_node",
        name="ekf_filter_node",
        output="screen"
    )
    ekf_process = rl.launch(ekf_node)
    rospy.loginfo("[robot_localization_node] ekf_localization_node started.")

    # 4) Start the PathPublisher (which will publish /robot_localization/path)
    path_pub = PathPublisher()

    # 5) Spin until shutdown
    rospy.spin()

    # On shutdown, stop the EKF node cleanly
    ekf_process.stop()
    rl.stop()
    rospy.loginfo("[robot_localization_node] Shutdown complete.")
