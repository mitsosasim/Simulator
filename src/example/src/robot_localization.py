#!/usr/bin/env python3
"""
robot_localization_node.py

A single ROS-Python script that:

  1. Converts your custom sensors into standard topics Robot Localization expects:
     • /automobile/localisation → /gps/fix (geometry_msgs/PoseWithCovarianceStamped)
     • /automobile/IMU         → /imu/data   (sensor_msgs/Imu)
     • /automobile/wheel_encoder/odometry remains nav_msgs/Odometry (no conversion)

  2. Launches Robot Localization’s ekf_localization_node in-process using roslaunch.scriptapi,
     with parameters that fuse (/automobile/wheel_encoder/odometry, /imu/data, /gps/fix).

  3. Subscribes to /odometry/filtered (EKF output) and republishes a nav_msgs/Path on
     /robot_localization/path for RViz visualization—starting from a fixed “true” start pose.

Usage:
  chmod +x robot_localization_node.py
  rosrun example robot_localization_node.py
"""

import rospy
import roslaunch
from tf.transformations import quaternion_from_euler
from utils_ros.msg import localisation, IMU as IMUMsg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path


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
    # Covariance: trust GPS to ~0.1 m in x,y; large for others
    cov = [0.01, 0,    0,    0,    0,    0,
           0,    0.01, 0,    0,    0,    0,
           0,    0,    1e6,  0,    0,    0,
           0,    0,    0,    1e6,  0,    0,
           0,    0,    0,    0,    1e6,  0,
           0,    0,    0,    0,    0,    1e6]
    gps.pose.covariance = cov
    gps_pub.publish(gps)


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
    # Moderate covariance for yaw
    imu.orientation_covariance = [1e-3, 0,    0,
                                  0,    1e-3, 0,
                                  0,    0,    1e-3]
    # No velocity/accel → set high covariance so they’re effectively ignored
    imu.angular_velocity_covariance = [1e6, 0, 0,
                                       0, 1e6, 0,
                                       0, 0, 1e6]
    imu.linear_acceleration_covariance = [1e6, 0, 0,
                                          0, 1e6, 0,
                                          0, 0, 1e6]
    imu_pub.publish(imu)


class PathPublisher:
    """
    Subscribes to /odometry/filtered → publishes /robot_localization/path,
    but pre-loads a “true” starting pose instead of (0,0,0).
    """
    def __init__(self):
        # Prepare the Path message
        self.path = Path()
        self.path.header.frame_id = "map"

        # 1) Insert a fixed “true” starting PoseStamped at init:
        init_ps = PoseStamped()
        init_ps.header.stamp = rospy.Time.now()
        init_ps.header.frame_id = "map"
        init_ps.pose.position.x = 0.8200948141321135
        init_ps.pose.position.y = -14.907983252855207
        init_ps.pose.position.z = 0.03293799834590568
        # Orientation not specified—use identity quaternion (zero rotation)
        init_ps.pose.orientation.x = 0.0
        init_ps.pose.orientation.y = 0.0
        init_ps.pose.orientation.z = 0.0
        init_ps.pose.orientation.w = 1.0

        self.path.header.stamp = init_ps.header.stamp
        self.path.poses.append(init_ps)

        # Publish that initial point immediately
        self.pub = rospy.Publisher("/robot_localization/path", Path, queue_size=1)
        rospy.sleep(0.1)
        self.pub.publish(self.path)

        # 2) Now subscribe to filtered odometry
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb, queue_size=10)

    def odom_cb(self, msg: Odometry):
        """
        Append each filtered odometry pose to the Path.
        """
        ps = PoseStamped()
        ps.header.stamp    = msg.header.stamp
        ps.header.frame_id = "map"
        ps.pose            = msg.pose.pose
        # Update header stamp and append
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(ps)
        self.pub.publish(self.path)


if __name__ == "__main__":
    rospy.init_node("robot_localization_node", anonymous=True)

    # Publishers for converted topics
    gps_pub = rospy.Publisher(
        "/gps/fix", PoseWithCovarianceStamped, queue_size=5
    )
    imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=5)

    # Subscribers to raw sensors
    rospy.Subscriber(
        "/automobile/localisation", localisation, localisation_cb, queue_size=5
    )
    rospy.Subscriber(
        "/automobile/IMU", IMUMsg, imu_cb, queue_size=5
    )
    # /automobile/wheel_encoder/odometry remains a nav_msgs/Odometry topic

    # 3) EKF parameters on the ROS parameter server
    ekf_params = {
        "ekf_filter_node/odom0": "/automobile/wheel_encoder/odometry",
        # 15 entries: [x,y,z,roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az]
        "ekf_filter_node/odom0_config": [
            False, False, False,
            False, False, False,
            True,  False, False,
            False, False, True,
            False, False, False
        ],
        "ekf_filter_node/odom0_queue_size": 10,

        "ekf_filter_node/imu0": "/imu/data",
        "ekf_filter_node/imu0_config": [
            False, False, False,
            False, False, True,   # yaw
            False, False, False,
            False, False, False,
            False, False, False
        ],
        "ekf_filter_node/imu0_queue_size": 10,

        "ekf_filter_node/gps0": "/gps/fix",
        "ekf_filter_node/gps0_config": [
            True,  True,  False,  # x,y
            False, False, False,
            False, False, False,
            False, False, False,
            False, False, False
        ],
        "ekf_filter_node/gps0_queue_size": 10,
        "ekf_filter_node/gps0_differential": False,
        "ekf_filter_node/gps0_relative": False,

        # Frame setup—use chassis::link instead of base_link
        "ekf_filter_node/base_link_frame": "chassis::link",
        "ekf_filter_node/odom_frame": "odom",
        "ekf_filter_node/map_frame": "map",
        "ekf_filter_node/publish_tf": False,

        "ekf_filter_node/frequency": 30.0,
        "ekf_filter_node/sensor_timeout": 0.1,
        "ekf_filter_node/stamped_control": False,

        "ekf_filter_node/process_noise_covariance": [
            1e-4, 0,    0,    0,    0,    0,
            0,    1e-4, 0,    0,    0,    0,
            0,    0,    1e-6, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6,
            0,    0,    0,    0,    0,    0,
            0,    0,    0,    0,    0,    0,
            0,    0,    0,    0,    0,    0,
            0,    0,    0,    0,    0,    0,
            0,    0,    0,    0,    0,    0,
            0,    0,    0,    0,    0,    0
        ],

        "ekf_filter_node/odom_noise_covariance": [
            1e-3, 0,    0,    0,    0,    0,
            0,    1e-3, 0,    0,    0,    0,
            0,    0,    1e-6, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ],

        "ekf_filter_node/imu_noise_covariance": [
            1e-6, 0,    0,    0,    0,    0,
            0,    1e-6, 0,    0,    0,    0,
            0,    0,    1e-4, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ],

        "ekf_filter_node/gps_noise_covariance": [
            1e-2, 0,    0,    0,    0,    0,
            0,    1e-2, 0,    0,    0,    0,
            0,    0,    1e-6, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ],
    }

    for param, value in ekf_params.items():
        rospy.set_param(param, value)

    # 4) Launch ekf_localization_node via roslaunch.scriptapi
    rl = roslaunch.scriptapi.ROSLaunch()
    rl.start()

    ekf_node = roslaunch.core.Node(
        package="robot_localization",
        node_type="ekf_localization_node",
        name="ekf_filter_node",
        output="screen"
    )
    process = rl.launch(ekf_node)
    rospy.loginfo("[robot_localization_node] ekf_localization_node started.")

    # 5) Start path publisher (will already contain that fixed “true” start point)
    path_pub = PathPublisher()

    # 6) Spin until shutdown
    rospy.spin()

    # 7) On shutdown, stop the EKF node
    process.stop()
    rl.stop()
    rospy.loginfo("[robot_localization_node] Shutdown complete.")
