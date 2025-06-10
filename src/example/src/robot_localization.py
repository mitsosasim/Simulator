#!/usr/bin/env python3
"""
robot_localization_compare.py

ROS Noetic Python node to launch a robot_localization EKF that fuses:
  - Wheel-encoder odometry (/automobile/wheel_encoder/odometry: nav_msgs/Odometry, twist.twist.linear.x = v, twist.twist.angular.z = ω; covariance set to var(v)=4e-4, var(ω)=1e-4)
  - IMU yaw (/imu/data: sensor_msgs/Imu converted from utils_ros/IMU; orientation covariance yaw var=9e-4; angular_velocity and linear_acceleration covariances large)
Initial pose seeded from /ground_truth_path. Publishes:
  - /robot_localization/path (nav_msgs/Path) in map frame for comparison.
  - Launches ekf_localization_node via roslaunch.scriptapi with parameters tuned similarly to your custom EKF noise.
  
Run this in parallel with your custom C++ EKF node, then compare /ekf/path vs /robot_localization/path to see the effect of visual updates.
"""

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from utils_ros.msg import IMU as CustomIMUMsg
import roslaunch
import math

class RobotLocalizationCompareNode:
    def __init__(self):
        rospy.init_node('robot_localization_compare_node', anonymous=False)

        # --- Initial pose from ground truth ---
        self.initial_pose_set = False
        self.init_x = None
        self.init_y = None
        self.init_z = None
        self.init_orientation = None  # geometry_msgs/Quaternion

        # --- First filtered reading for path remapping ---
        self.first_filt_set = False
        self.first_filt_x = None
        self.first_filt_y = None

        # Path message for fused trajectory
        self.path = Path()
        self.path.header.frame_id = "map"

        # Publishers
        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=5)
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                                               queue_size=1, latch=True)
        self.path_pub = rospy.Publisher("/robot_localization/path", Path, queue_size=1)

        # Subscribers
        self.imu_sub = rospy.Subscriber("/automobile/IMU", CustomIMUMsg, self.imu_cb, queue_size=5)
        self.gt_sub = rospy.Subscriber("/ground_truth_path", Path, self.ground_truth_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry,
                                         self.odom_cb, queue_size=10)

        # Broadcast static identity transform map->odom
        self.publish_static_map_to_odom()

        # Set robot_localization EKF parameters
        self.set_ekf_parameters()

        # Launch EKF node
        self.launch_handle = None
        self.launch_ekf_node()

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[robot_localization_compare] Node initialized; spinning...")

    def publish_static_map_to_odom(self):
        """Broadcast one-time static identity transform map -> odom."""
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        from geometry_msgs.msg import TransformStamped
        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "map"
        static_tf.child_frame_id = "odom"
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        static_broadcaster.sendTransform(static_tf)
        rospy.loginfo("[robot_localization_compare] Broadcast static identity map->odom")

    def set_ekf_parameters(self):
        """
        Push parameters for ekf_localization_node onto ROS parameter server under namespace ekf_filter_node.
        Noise values are chosen to mirror the C++ EKF:
          - Wheel odometry: σ_v=0.02 m/s ⇒ var=4e-4; σ_ω=0.01 rad/s ⇒ var=1e-4.
            We assume the Odometry message's twist.twist.covariance is set accordingly by the publisher.
          - IMU yaw: σ_yaw=0.03 rad ⇒ var=9e-4.
          - Process noise on pose: we use small values (e.g. 1e-4) for x,y; orientation small.
        """
        ns = "ekf_filter_node"

        # 1) Wheel-Encoder Odometry input (odom0)
        rospy.set_param(f"{ns}/odom0", "/automobile/wheel_encoder/odometry")
        # Mask: fuse vx and yaw rate only
        odom0_config = [
            False, False, False,   # x, y, z pose (not fusing pose)
            False, False, False,   # roll, pitch, yaw pose
            True, False, False,    # vx = forward speed
            False, False, True,    # vyaw = yaw rate
            False, False, False    # ax, ay, az not used
        ]
        rospy.set_param(f"{ns}/odom0_config", odom0_config)
        rospy.set_param(f"{ns}/odom0_queue_size", 10)
        # Odometry measurement noise covariance (6×6 for pose part). 
        # Since we do not fuse pose from odometry, these values are less critical, but must be set.
        # Set diagonal small for x,y but EKF won't fuse them:
        odom_noise_cov = [
            1e-2, 0,    0,    0,    0,    0,
            0,    1e-2, 0,    0,    0,    0,
            0,    0,    1e-3, 0,    0,    0,
            0,    0,    0,    1e-3, 0,    0,
            0,    0,    0,    0,    1e-3, 0,
            0,    0,    0,    0,    0,    1e-3
        ]
        rospy.set_param(f"{ns}/odom_noise_covariance", odom_noise_cov)

        # 2) IMU orientation input (imu0)
        rospy.set_param(f"{ns}/imu0", "/imu/data")
        # Fuse yaw only
        imu0_config = [
            False, False, False,   # x, y, z pose
            False, False, True,    # yaw pose from IMU orientation
            False, False, False,   # vx, vy, vz
            False, False, False,   # vroll, vpitch, vyaw (we rely on wheel odom yaw rate)
            False, False, False    # ax, ay, az
        ]
        rospy.set_param(f"{ns}/imu0_config", imu0_config)
        rospy.set_param(f"{ns}/imu0_queue_size", 10)
        # IMU orientation measurement noise covariance (6×6: [x,y,z,roll,pitch,yaw]).
        # We only fuse yaw, so yaw variance = 9e-4. For others, small or irrelevant.
        imu_noise_cov = [
            1e-6, 0,    0,    0,    0,    0,
            0,    1e-6, 0,    0,    0,    0,
            0,    0,    1e-3, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    9e-4   # yaw variance
        ]
        rospy.set_param(f"{ns}/imu_noise_covariance", imu_noise_cov)

        # 3) Frames
        rospy.set_param(f"{ns}/base_link_frame", "chassis::link")
        rospy.set_param(f"{ns}/odom_frame", "odom")
        rospy.set_param(f"{ns}/map_frame", "map")
        # Publish map->odom TF
        rospy.set_param(f"{ns}/publish_tf", True)
        # Allow reset on /initialpose
        rospy.set_param(f"{ns}/reset_on_pose", True)
        # EKF frequency
        rospy.set_param(f"{ns}/frequency", 30.0)
        # Sensor timeout
        rospy.set_param(f"{ns}/sensor_timeout", 0.1)
        rospy.set_param(f"{ns}/stamped_control", False)

        # 4) Process noise covariance (36 entries: 6×6 for pose states, then zeros)
        # We set pose-process noise: x,y ~1e-4, z small, roll/pitch small, yaw small
        proc6 = [
            1e-4, 0,    0,    0,    0,    0,
            0,    1e-4, 0,    0,    0,    0,
            0,    0,    1e-6, 0,    0,    0,
            0,    0,    0,    1e-6, 0,    0,
            0,    0,    0,    0,    1e-6, 0,
            0,    0,    0,    0,    0,    1e-6
        ]
        process_noise = proc6 + [0.0]*30
        rospy.set_param(f"{ns}/process_noise_covariance", process_noise)

        rospy.loginfo("[robot_localization_compare] Set EKF parameters under 'ekf_filter_node/'")

    def launch_ekf_node(self):
        """Launch ekf_localization_node via roslaunch.scriptapi after parameters are set."""
        rospy.loginfo("[robot_localization_compare] Launching ekf_localization_node...")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        node = roslaunch.core.Node(package="robot_localization",
                                   node_type="ekf_localization_node",
                                   name="ekf_filter_node",
                                   output="screen")
        try:
            launch.launch(node)
            self.launch_handle = launch
            rospy.loginfo("[robot_localization_compare] ekf_localization_node launched.")
        except Exception as e:
            rospy.logerr("[robot_localization_compare] Failed to launch ekf_localization_node: %s", e)

    def imu_cb(self, msg):
        """
        Convert utils_ros/IMU (roll,pitch,yaw) into sensor_msgs/Imu on /imu/data.
        Orientation covariance yaw var=9e-4, other covariances small for orientation,
        angular_velocity_cov and linear_accel_cov set very large.
        """
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "chassis::link"

        # roll, pitch, yaw from custom
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        quat = quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation = Quaternion(*quat)

        # orientation covariance: only yaw fused; but we set full 3x3
        imu_msg.orientation_covariance = [1e-3, 0.0, 0.0,
                                          0.0, 1e-3, 0.0,
                                          0.0, 0.0, 9e-4]
        # No real angular velocity or linear accel: large covariance
        large = 1e6
        imu_msg.angular_velocity_covariance = [large]*9
        imu_msg.linear_acceleration_covariance = [large]*9

        self.imu_pub.publish(imu_msg)

    def ground_truth_cb(self, msg):
        """
        Read first PoseStamped from /ground_truth_path, store x,y,z and orientation,
        publish /initialpose to seed EKF. Then unregister.
        """
        if self.initial_pose_set:
            try:
                self.gt_sub.unregister()
            except:
                pass
            return

        if not msg.poses:
            rospy.logwarn("[robot_localization_compare] /ground_truth_path empty, waiting...")
            return

        first = msg.poses[0]
        p = first.pose.position
        ori = first.pose.orientation
        self.init_x = p.x
        self.init_y = p.y
        self.init_z = p.z
        self.init_orientation = ori
        self.initial_pose_set = True

        # Log yaw for debugging
        yaw = self.quaternion_to_yaw(ori)
        rospy.loginfo("[robot_localization_compare] Ground-truth initial pose: x=%.3f y=%.3f z=%.3f yaw=%.1f°",
                      self.init_x, self.init_y, self.init_z, yaw*180.0/math.pi)

        # Publish PoseWithCovarianceStamped on /initialpose
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.stamp = rospy.Time.now()
        init_msg.header.frame_id = "map"
        init_msg.pose.pose.position = Point(self.init_x, self.init_y, self.init_z)
        init_msg.pose.pose.orientation = ori
        # Covariance: small to trust initial
        pos_var = 1e-4
        ori_var = 1e-4
        cov = [
            pos_var, 0.0,     0.0,     0.0,     0.0,     0.0,
            0.0,     pos_var, 0.0,     0.0,     0.0,     0.0,
            0.0,     0.0,     pos_var, 0.0,     0.0,     0.0,
            0.0,     0.0,     0.0,     ori_var, 0.0,     0.0,
            0.0,     0.0,     0.0,     0.0,     ori_var, 0.0,
            0.0,     0.0,     0.0,     0.0,     0.0,     ori_var
        ]
        init_msg.pose.covariance = cov
        self.initialpose_pub.publish(init_msg)
        rospy.loginfo("[robot_localization_compare] Published /initialpose to seed EKF.")

    def odom_cb(self, msg):
        """
        Remap /odometry/filtered into /robot_localization/path in map frame:
        - Wait until initial pose set.
        - Discard (0,0) until first non-zero.
        - On first valid reading, record first_filt_x,y and append initial ground-truth pose (with ground-truth orientation) to path.
        - For subsequent readings, compute dx,dy relative to first filtered, then mapped_x = init_x + dx, mapped_y = init_y + dy, z=init_z; orientation = filtered orientation.
        """
        if not self.initial_pose_set:
            return

        pos = msg.pose.pose.position
        x_f = pos.x
        y_f = pos.y

        if not self.first_filt_set:
            if abs(x_f) < 1e-6 and abs(y_f) < 1e-6:
                return
            # First valid
            self.first_filt_x = x_f
            self.first_filt_y = y_f
            self.first_filt_set = True
            rospy.loginfo("[robot_localization_compare] First non-zero filtered reading: x=%.3f y=%.3f; seeding path.",
                          x_f, y_f)
            seed = PoseStamped()
            seed.header.stamp = rospy.Time.now()
            seed.header.frame_id = "map"
            seed.pose.position = Point(self.init_x, self.init_y, self.init_z)
            seed.pose.orientation = self.init_orientation
            self.path.poses.append(seed)
            self.path.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path)
            return

        # Subsequent
        dx = x_f - self.first_filt_x
        dy = y_f - self.first_filt_y
        mx = self.init_x + dx
        my = self.init_y + dy
        mz = self.init_z

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position = Point(mx, my, mz)
        ps.pose.orientation = msg.pose.pose.orientation
        self.path.poses.append(ps)
        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    def quaternion_to_yaw(self, q):
        """Compute yaw from geometry_msgs/Quaternion."""
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return math.atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def on_shutdown(self):
        rospy.loginfo("[robot_localization_compare] Shutdown: terminating EKF node.")
        if self.launch_handle is not None:
            try:
                self.launch_handle.shutdown()
                rospy.loginfo("[robot_localization_compare] EKF node shut down.")
            except Exception as e:
                rospy.logwarn("[robot_localization_compare] Exception on shutdown: %s", e)

def main():
    node = RobotLocalizationCompareNode()
    rospy.spin()

if __name__ == "__main__":
    main()
