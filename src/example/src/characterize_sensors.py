#!/usr/bin/env python3
"""
characterize_sensors.py

Collects sensor data from:
  - /ground_truth_path (nav_msgs/Path) for ground-truth poses
  - /automobile/wheel_encoder/odometry (nav_msgs/Odometry) for encoder-based v, ω
  - /automobile/IMU (utils_ros/IMU) for yaw readings

Buffers odom/IMU and GT, interpolates GT for each sensor timestamp once GT bracketing data is available,
and computes residual statistics on shutdown.

Handles small timestamp mismatches by allowing a tolerance window beyond the latest GT.
"""
import rospy
import math
import threading
import numpy as np
from collections import deque
from nav_msgs.msg import Path, Odometry
from utils_ros.msg import IMU as IMUMsg
import tf
import bisect

class SensorCharacterizer:
    def __init__(self):
        rospy.init_node('sensor_characterizer', anonymous=True, log_level=rospy.DEBUG)

        # Parameters
        self.max_buffer_secs = rospy.get_param('~max_buffer_secs', 10.0)
        self.min_gt_buffer_len = rospy.get_param('~min_buffer_len', 2)
        self.tolerance = rospy.get_param('~time_tolerance', 0.03)  # seconds tolerance for slight overshoot

        # Buffers
        self.gt_buffer = deque()
        self.gt_lock = threading.Lock()
        self.odom_buffer = deque()
        self.odom_lock = threading.Lock()
        self.imu_buffer = deque()
        self.imu_lock = threading.Lock()

        self.gt_time_offset = None
        self.got_first_gt = False

        # Residual storage
        self.velocity_residuals = []
        self.yawrate_residuals = []
        self.imu_yaw_residuals = []

        # Subscribers
        rospy.Subscriber('/ground_truth_path', Path, self.gt_callback, queue_size=10)
        rospy.Subscriber('/automobile/wheel_encoder/odometry', Odometry, self.odom_callback, queue_size=100)
        rospy.Subscriber('/automobile/IMU', IMUMsg, self.imu_callback, queue_size=50)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[SensorCharacterizer] Started. Buffering from /ground_truth_path, /automobile/wheel_encoder/odometry, /automobile/IMU")
        rospy.loginfo(f"[SensorCharacterizer] max_buffer_secs={self.max_buffer_secs}, min_gt_buffer_len={self.min_gt_buffer_len}, tolerance={self.tolerance}")

    def gt_callback(self, path_msg):
        if not path_msg.poses:
            rospy.logdebug("[SensorCharacterizer] Received empty GT Path.")
            return
        last_pose = path_msg.poses[-1]
        if last_pose.header.stamp is None:
            rospy.logwarn("[SensorCharacterizer] GT PoseStamped has no header.stamp; skipping.")
            return
        t_hdr = last_pose.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        if not self.got_first_gt:
            self.gt_time_offset = now - t_hdr
            self.got_first_gt = True
            rospy.loginfo(f"[SensorCharacterizer] First GT: header.stamp={t_hdr:.3f}, now={now:.3f}, gt_time_offset={self.gt_time_offset:.3f}")
        t_adj = t_hdr + (self.gt_time_offset or 0.0)
        x = last_pose.pose.position.x
        y = last_pose.pose.position.y
        q = last_pose.pose.orientation
        try:
            _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception as e:
            rospy.logwarn(f"[SensorCharacterizer] Failed yaw extraction from GT: {e}")
            return

        with self.gt_lock:
            if self.gt_buffer and abs(self.gt_buffer[-1][0] - t_adj) < 1e-6:
                rospy.logdebug(f"[SensorCharacterizer] GT duplicate t_adj={t_adj:.3f}, skipping.")
            else:
                self.gt_buffer.append((t_adj, x, y, yaw))
                rospy.logdebug(f"[SensorCharacterizer] GT buffered: t_adj={t_adj:.3f}, x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}. size={len(self.gt_buffer)}")
            # prune old
            while self.gt_buffer and (t_adj - self.gt_buffer[0][0] > self.max_buffer_secs):
                removed = self.gt_buffer.popleft()
                rospy.logdebug(f"[SensorCharacterizer] GT prune old t={removed[0]:.3f}")
        # Attempt processing
        self.process_buffered_odom_and_imu()

    def interpolate_gt(self, t_query):
        buf = self.gt_buffer
        if len(buf) < self.min_gt_buffer_len:
            rospy.logdebug("[SensorCharacterizer] GT buffer too small for interpolation.")
            return None
        times = [entry[0] for entry in buf]
        idx = bisect.bisect_left(times, t_query)
        if idx == 0 or idx >= len(buf):
            rospy.logdebug(f"[SensorCharacterizer] t_query={t_query:.3f} outside GT range [{times[0]:.3f}, {times[-1]:.3f}].")
            return None
        t1, x1, y1, yaw1 = buf[idx-1]
        t2, x2, y2, yaw2 = buf[idx]
        dt = t2 - t1
        if dt <= 1e-6:
            return x1, y1, yaw1
        alpha = (t_query - t1)/dt
        xi = x1 + alpha*(x2 - x1)
        yi = y1 + alpha*(y2 - y1)
        dyaw = yaw2 - yaw1
        while dyaw > math.pi: dyaw -= 2.0*math.pi
        while dyaw < -math.pi: dyaw += 2.0*math.pi
        yawi = yaw1 + alpha*dyaw
        while yawi > math.pi: yawi -= 2.0*math.pi
        while yawi < -math.pi: yawi += 2.0*math.pi
        rospy.logdebug(f"[SensorCharacterizer] Interpolated GT at {t_query:.3f}: x={xi:.3f}, y={yi:.3f}, yaw={yawi:.3f} between {t1:.3f}-{t2:.3f}")
        return xi, yi, yawi

    def odom_callback(self, odom_msg):
        if odom_msg.header is None or odom_msg.header.stamp is None:
            rospy.logwarn("[SensorCharacterizer] Odometry has no header.stamp; using now().")
            t = rospy.Time.now().to_sec()
        else:
            t = odom_msg.header.stamp.to_sec()
        meas_v = odom_msg.twist.twist.linear.x
        meas_w = odom_msg.twist.twist.angular.z
        with self.odom_lock:
            self.odom_buffer.append((t, meas_v, meas_w))
            rospy.logdebug(f"[SensorCharacterizer] Buffered odom: t={t:.3f}, v={meas_v:.3f}, w={meas_w:.3f}, buffer size={len(self.odom_buffer)}")
        self.process_buffered_odom_and_imu()

    def imu_callback(self, imu_msg):
        try:
            t = imu_msg.header.stamp.to_sec()
        except Exception:
            t = rospy.Time.now().to_sec()
            rospy.logdebug("[SensorCharacterizer] IMU no header.stamp; using now().")
        if hasattr(imu_msg, 'yaw'):
            meas_yaw = imu_msg.yaw
        elif hasattr(imu_msg, 'orientation'):
            q = imu_msg.orientation
            try:
                _, _, meas_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            except Exception as e:
                rospy.logwarn(f"[SensorCharacterizer] Failed to extract yaw from IMU quaternion: {e}")
                return
        else:
            rospy.logwarn("[SensorCharacterizer] IMU msg has no yaw/orientation; skipping.")
            return
        with self.imu_lock:
            self.imu_buffer.append((t, meas_yaw))
            rospy.logdebug(f"[SensorCharacterizer] Buffered IMU: t={t:.3f}, yaw={meas_yaw:.3f}, buffer size={len(self.imu_buffer)}")
        self.process_buffered_odom_and_imu()

    def process_buffered_odom_and_imu(self):
        with self.gt_lock:
            if len(self.gt_buffer) < self.min_gt_buffer_len:
                return
            times = [entry[0] for entry in self.gt_buffer]
            t_min, t_max = times[0], times[-1]

        # Process odometry buffer
        to_remove = []
        with self.odom_lock:
            for idx, (t, meas_v, meas_w) in enumerate(self.odom_buffer):
                if t < t_min:
                    rospy.logdebug(f"[SensorCharacterizer] Odometry at t={t:.3f} < GT start {t_min:.3f}; discarding.")
                    to_remove.append(idx)
                elif t > t_max:
                    # allow small tolerance
                    if t <= t_max + self.tolerance:
                        with self.gt_lock:
                            t_gt_last, x_gt_last, y_gt_last, yaw_gt_last = self.gt_buffer[-1]
                        rospy.logdebug(f"[SensorCharacterizer] Odometry at t={t:.3f} slightly >GT latest {t_max:.3f}; using last GT.")
                        if not hasattr(self, 'last_processed_odom'):
                            self.last_processed_odom = (t, x_gt_last, y_gt_last, yaw_gt_last)
                            rospy.logdebug(f"[SensorCharacterizer] Stored first processed odom at t={t:.3f}")
                        else:
                            t_prev, x_prev, y_prev, yaw_prev = self.last_processed_odom
                            dt = t - t_prev
                            if dt > 1e-3:
                                dx = x_gt_last - x_prev
                                dy = y_gt_last - y_prev
                                v_gt = math.hypot(dx, dy)/dt
                                dyaw = yaw_gt_last - yaw_prev
                                while dyaw > math.pi: dyaw -= 2.0*math.pi
                                while dyaw < -math.pi: dyaw += 2.0*math.pi
                                w_gt = dyaw/dt
                                res_v = meas_v - v_gt
                                res_w = meas_w - w_gt
                                self.velocity_residuals.append(res_v)
                                self.yawrate_residuals.append(res_w)
                                rospy.logdebug(f"[SensorCharacterizer] Odom residual (tol): meas_v={meas_v:.3f}, v_gt={v_gt:.3f}, res_v={res_v:.3f}; meas_w={meas_w:.3f}, w_gt={w_gt:.3f}, res_w={res_w:.3f}")
                                self.last_processed_odom = (t, x_gt_last, y_gt_last, yaw_gt_last)
                            else:
                                rospy.logdebug(f"[SensorCharacterizer] Odom dt too small dt={dt:.6f}; skipping residual.")
                        to_remove.append(idx)
                        continue
                    else:
                        rospy.logdebug(f"[SensorCharacterizer] Odometry at t={t:.3f} > GT latest {t_max:.3f}+tol; will wait.")
                        break
                else:
                    # t_min <= t <= t_max: interpolate
                    with self.gt_lock:
                        gt_interp = self.interpolate_gt(t)
                    if gt_interp is not None:
                        xi, yi, yawi = gt_interp
                        if not hasattr(self, 'last_processed_odom'):
                            self.last_processed_odom = (t, xi, yi, yawi)
                            rospy.logdebug(f"[SensorCharacterizer] Stored first processed odom at t={t:.3f}")
                        else:
                            t_prev, x_prev, y_prev, yaw_prev = self.last_processed_odom
                            dt = t - t_prev
                            if dt > 1e-3:
                                dx = xi - x_prev
                                dy = yi - y_prev
                                v_gt = math.hypot(dx, dy)/dt
                                dyaw = yawi - yaw_prev
                                while dyaw > math.pi: dyaw -= 2.0*math.pi
                                while dyaw < -math.pi: dyaw += 2.0*math.pi
                                w_gt = dyaw/dt
                                res_v = meas_v - v_gt
                                res_w = meas_w - w_gt
                                self.velocity_residuals.append(res_v)
                                self.yawrate_residuals.append(res_w)
                                rospy.logdebug(f"[SensorCharacterizer] Odom residual: meas_v={meas_v:.3f}, v_gt={v_gt:.3f}, res_v={res_v:.3f}; meas_w={meas_w:.3f}, w_gt={w_gt:.3f}, res_w={res_w:.3f}")
                                self.last_processed_odom = (t, xi, yi, yawi)
                            else:
                                rospy.logdebug(f"[SensorCharacterizer] Odom dt too small dt={dt:.6f}; skipping residual.")
                        to_remove.append(idx)
                    else:
                        rospy.logdebug(f"[SensorCharacterizer] Odometry at t={t:.3f} interpolation failed; discarding.")
                        to_remove.append(idx)
            # remove processed/discarded
            for idx in reversed(to_remove):
                try:
                    self.odom_buffer.remove(self.odom_buffer[idx])
                except Exception:
                    pass

        # Process IMU buffer
        to_remove_imu = []
        with self.imu_lock:
            for idx, (t, meas_yaw) in enumerate(self.imu_buffer):
                if t < t_min:
                    rospy.logdebug(f"[SensorCharacterizer] IMU at t={t:.3f} < GT start; discarding.")
                    to_remove_imu.append(idx)
                elif t > t_max:
                    if t <= t_max + self.tolerance:
                        with self.gt_lock:
                            _, _, _, yaw_gt_last = self.gt_buffer[-1]
                        rospy.logdebug(f"[SensorCharacterizer] IMU at t={t:.3f} slightly >GT latest; using last GT yaw.")
                        dyaw = meas_yaw - yaw_gt_last
                        while dyaw > math.pi: dyaw -= 2.0*math.pi
                        while dyaw < -math.pi: dyaw += 2.0*math.pi
                        self.imu_yaw_residuals.append(dyaw)
                        rospy.logdebug(f"[SensorCharacterizer] IMU residual (tol): meas_yaw={meas_yaw:.3f}, yaw_gt={yaw_gt_last:.3f}, res={dyaw:.3f}")
                        to_remove_imu.append(idx)
                        continue
                    else:
                        rospy.logdebug(f"[SensorCharacterizer] IMU at t={t:.3f} > GT latest+tol; will wait.")
                        break
                else:
                    with self.gt_lock:
                        gt_interp = self.interpolate_gt(t)
                    if gt_interp is not None:
                        _, _, yawi = gt_interp
                        dyaw = meas_yaw - yawi
                        while dyaw > math.pi: dyaw -= 2.0*math.pi
                        while dyaw < -math.pi: dyaw += 2.0*math.pi
                        self.imu_yaw_residuals.append(dyaw)
                        rospy.logdebug(f"[SensorCharacterizer] IMU residual: meas_yaw={meas_yaw:.3f}, yaw_gt={yawi:.3f}, res={dyaw:.3f}")
                        to_remove_imu.append(idx)
                    else:
                        rospy.logdebug(f"[SensorCharacterizer] IMU at t={t:.3f} interpolation failed; discarding.")
                        to_remove_imu.append(idx)
            for idx in reversed(to_remove_imu):
                try:
                    self.imu_buffer.remove(self.imu_buffer[idx])
                except Exception:
                    pass

    def on_shutdown(self):
        rospy.loginfo("[SensorCharacterizer] Shutdown detected. Processing collected data...")
        v_res = np.array(self.velocity_residuals) if self.velocity_residuals else np.array([])
        w_res = np.array(self.yawrate_residuals) if self.yawrate_residuals else np.array([])
        imu_res = np.array(self.imu_yaw_residuals) if self.imu_yaw_residuals else np.array([])

        def process_residuals(res, name):
            if res.size == 0:
                rospy.logwarn(f"[SensorCharacterizer] No residuals collected for {name}.")
                return None, None
            res = res[~np.isnan(res)]
            if res.size == 0:
                rospy.logwarn(f"[SensorCharacterizer] All residuals NaN for {name}.")
                return None, None
            mu = np.mean(res)
            sigma = np.std(res)
            if sigma <= 0:
                rospy.logwarn(f"[SensorCharacterizer] Zero std for {name}.")
                return mu, sigma
            mask = np.abs(res - mu) <= 3.0*sigma
            res_filt = res[mask]
            mu_f = np.mean(res_filt) if res_filt.size>0 else mu
            sigma_f = np.std(res_filt) if res_filt.size>0 else sigma
            rospy.loginfo(f"[SensorCharacterizer] {name}: raw count={res.size}, kept={res_filt.size}, mean={mu_f:.6f}, std={sigma_f:.6f}")
            return mu_f, sigma_f

        rospy.loginfo("=== Sensor Characterization Results ===")
        mu_v, std_v = process_residuals(v_res, "Wheel-encoder velocity residual")
        mu_w, std_w = process_residuals(w_res, "Wheel-encoder yaw-rate residual")
        mu_imu, std_imu = process_residuals(imu_res, "IMU yaw residual")
        rospy.loginfo("=== End of characterization ===")
        if std_v is not None:
            rospy.loginfo(f"=> Suggest R_odom_(0,0) variance ≈ {std_v**2:.6e}")
        if std_w is not None:
            rospy.loginfo(f"=> Suggest R_odom_(1,1) variance ≈ {std_w**2:.6e}")
        if std_imu is not None:
            rospy.loginfo(f"=> Suggest R_imu_ variance ≈ {std_imu**2:.6e}")

def main():
    SensorCharacterizer()
    rospy.spin()

if __name__ == '__main__':
    main()
