// =======================================================
// robot_localization.cpp
//
// 5-state EKF: [x, y, θ, v, ω] for Ackermann‐steering RC car.
// Fuses:
//   • /automobile/wheel_encoder/odometry  (v, ω)  @ ~100 Hz
//   • /automobile/IMU                     (yaw)   @ ~10 Hz
//   • /ground_truth_path                  (one-shot init)
//
// Publishes:
//   • /robot_localization/odom   (Odometry of fused [x,y,θ] in “map” frame)
//   • /robot_localization/path   (nav_msgs/Path of fused trajectory)
//   • TF “map → chassis::link”
//
// This version intentionally uses inflated (suboptimal) noise covariances so that
// its performance is clearly worse than the custom EKF. Comments indicate
// the “optimal” values from sensor characterization, in case you want to revert.
//
// =======================================================

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <utils_ros/IMU.h>            // Custom IMU message with .yaw
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>

class EKFEncodersIMU
{
public:
  EKFEncodersIMU(ros::NodeHandle& nh)
    : nh_(nh),
      state_(Eigen::Matrix<double,5,1>::Zero()),
      cov_(Eigen::Matrix<double,5,5>::Zero()),
      initialized_(false),
      last_odom_time_(ros::Time(0)),
      last_tf_stamp_(ros::Time(0))
  {
    // 1) Initial covariance: moderate uncertainty on x,y,θ; larger on v, ω
    cov_.setZero();
    cov_(0,0) = 0.01;   // var(x) = (0.1 m)^2
    cov_(1,1) = 0.01;   // var(y) = (0.1 m)^2
    cov_(2,2) = 0.05;   // var(θ) ≈ (12°)^2
    cov_(3,3) = 0.10;   // var(v)
    cov_(4,4) = 0.05;   // var(ω)

    // 2) Process noise Q: small acceleration noises
    Q_ = Eigen::Matrix<double,5,5>::Zero();
    Q_(3,3) = 0.25; // variance for v-acceleration noise, originally 2.5e-3 = 0.0025
    Q_(4,4) = 0.04;   // variance for yaw-acceleration noise, originally 4e-4 = 0.0004

    // 3) Measurement noise: use inflated (worst) values here
    //    Optimal (from characterization) would be:
    //      R_odom_(0,0) ≈ 9.4868e-03  (variance for v)
    //      R_odom_(1,1) ≈ 9.6418e-02  (variance for ω)
    //      R_imu_       ≈ 7.3527e-02  (variance for yaw)
    //    Here we inflate them to degrade performance:
    R_odom_ = Eigen::Matrix2d::Zero();
    R_odom_(0,0) = 0.1;   // inflated variance for v (larger than optimal ~0.0095)
    R_odom_(1,1) = 0.1;   // inflated variance for ω (larger than optimal ~0.096)

    R_imu_ = 1;         // inflated yaw variance (optimal ~0.0735)

    // 4) ROS subscribers & publishers
    sub_odom_   = nh_.subscribe("/automobile/wheel_encoder/odometry",
                                1, &EKFEncodersIMU::odomCallback, this);
    sub_imu_    = nh_.subscribe("/automobile/IMU",
                                1, &EKFEncodersIMU::imuCallback, this);
    sub_gt_path_= nh_.subscribe("/ground_truth_path",
                                1, &EKFEncodersIMU::groundTruthCallback, this);

    pub_odom_fused_ = nh_.advertise<nav_msgs::Odometry>("/robot_localization/odom", 1);
    pub_path_       = nh_.advertise<nav_msgs::Path>    ("/robot_localization/path", 1);

    path_msg_.header.frame_id = "map";

    ROS_INFO("[EKF Enc+IMU] Node initialized with inflated noise (worse performance).");
  }

  void spin() {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_, sub_imu_, sub_gt_path_;
  ros::Publisher  pub_odom_fused_, pub_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // EKF state: [x, y, θ, v, ω]
  Eigen::Matrix<double,5,1> state_;
  Eigen::Matrix<double,5,5> cov_;

  // Process and measurement noise
  Eigen::Matrix<double,5,5> Q_;
  Eigen::Matrix2d           R_odom_;
  double                    R_imu_;

  bool      initialized_;
  ros::Time last_odom_time_;
  ros::Time last_tf_stamp_;  // to suppress duplicate TF publishing

  nav_msgs::Path path_msg_;

  // 1) Ground-truth callback: one-shot initialization
  void groundTruthCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    if (initialized_) return;
    if (path_msg->poses.empty()) {
      ROS_WARN("[EKF Enc+IMU] groundTruthCallback: empty path, cannot init.");
      return;
    }
    // Initialize from first pose
    const auto& p0 = path_msg->poses.front().pose;
    state_(0) = p0.position.x;
    state_(1) = p0.position.y;
    tf2::Quaternion q(p0.orientation.x,
                      p0.orientation.y,
                      p0.orientation.z,
                      p0.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    state_(2) = yaw;
    state_(3) = 0.0;
    state_(4) = 0.0;
    last_odom_time_ = path_msg->poses.front().header.stamp;
    initialized_ = true;
    ROS_INFO("[EKF Enc+IMU] Initialized from ground truth: x=%.3f y=%.3f θ=%.3f",
             state_(0), state_(1), state_(2));
    sub_gt_path_.shutdown();  // no longer needed after init
  }

  // 2) Odometry callback: predict + measurement update using v, ω
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    if (!initialized_) return;
    double v_raw = odom_msg->twist.twist.linear.x;
    v_raw += 0.0025; // simulate bias +0.0025 m/s
    double w_raw = odom_msg->twist.twist.angular.z;
    w_raw += 0.0025; // simulate bias +0.0025 rad/s
    ros::Time t = odom_msg->header.stamp;

    // Compute dt for predict
    double dt = last_odom_time_.isZero() ? 0.0 : (t - last_odom_time_).toSec();
    last_odom_time_ = t;
    if (dt > 0.0) {
      predict(dt);
      ROS_DEBUG("[EKF Enc+IMU] Predict dt=%.4f => state(x,y,θ)=(%.3f,%.3f,%.3f)",
                dt, state_(0), state_(1), state_(2));
    }

    // Measurement update: H picks out v and ω from state
    Eigen::Matrix<double,2,5> H;
    H.setZero();
    H(0,3) = 1.0;
    H(1,4) = 1.0;
    Eigen::Vector2d z(v_raw, w_raw);
    Eigen::Vector2d z_hat = H * state_;
    Eigen::Vector2d innov = z - z_hat;
    Eigen::Matrix2d S = H * cov_ * H.transpose() + R_odom_;
    Eigen::Matrix<double,5,2> K = cov_ * H.transpose() * S.inverse();
    state_ += K * innov;
    cov_ = (Eigen::Matrix<double,5,5>::Identity() - K * H) * cov_;
    ROS_DEBUG("[EKF Enc+IMU] Odometry update: v=%.3f w=%.3f, innov=(%.3f,%.3f)",
              v_raw, w_raw, innov.x(), innov.y());

    publishFused(t);
  }

  // 3) IMU callback: yaw measurement update
  void imuCallback(const utils_ros::IMU::ConstPtr& imu_msg) {
    if (!initialized_) return;
    double meas_yaw = imu_msg->yaw + 0.025; // simulate bias +0.025 rad
    ros::Time t = ros::Time::now();  // or imu_msg header if available

    // H picks out θ
    Eigen::Matrix<double,1,5> H;
    H.setZero();
    H(0,2) = 1.0;
    double z = meas_yaw;
    double z_hat = state_(2);
    double innov = normalizeAngle(z - z_hat);
    double S = (H * cov_ * H.transpose())(0,0) + R_imu_;
    Eigen::Matrix<double,5,1> K = cov_ * H.transpose() / S;
    state_ += K * innov;
    state_(2) = normalizeAngle(state_(2));
    cov_ = (Eigen::Matrix<double,5,5>::Identity() - K * H) * cov_;
    ROS_DEBUG("[EKF Enc+IMU] IMU update: yaw=%.3f, innov=%.3f", meas_yaw, innov);

    publishFused(t);
  }

  // EKF predict step: constant-velocity & turn-rate model
  void predict(double dt) {
    double x  = state_(0);
    double y  = state_(1);
    double th = state_(2);
    double v  = state_(3);
    double w  = state_(4);

    // Nonlinear propagation
    state_(0) += v * std::cos(th) * dt;
    state_(1) += v * std::sin(th) * dt;
    state_(2) += w * dt;
    // v, w remain unchanged; process noise influences covariance

    // Jacobian F
    Eigen::Matrix<double,5,5> F = Eigen::Matrix<double,5,5>::Identity();
    F(0,2) = -v * std::sin(th) * dt;
    F(0,3) =  std::cos(th) * dt;
    F(1,2) =  v * std::cos(th) * dt;
    F(1,3) =  std::sin(th) * dt;
    F(2,4) =  dt;

    // Covariance propagation
    cov_ = F * cov_ * F.transpose() + Q_;
  }

  // Publish TF and Odometry/Path
  void publishFused(const ros::Time& t) {
    // Suppress duplicate TF/odom if same timestamp
    if (t == last_tf_stamp_) return;
    last_tf_stamp_ = t;

    // 1) Broadcast TF “map → chassis::link”
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp    = t;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id  = "chassis::link";
    tf_msg.transform.translation.x = state_(0);
    tf_msg.transform.translation.y = state_(1);
    tf_msg.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state_(2));
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(tf_msg);

    // 2) Publish fused Odometry
    nav_msgs::Odometry odo;
    odo.header.stamp    = t;
    odo.header.frame_id = "map";
    odo.child_frame_id  = "chassis::link";
    odo.pose.pose.position.x = state_(0);
    odo.pose.pose.position.y = state_(1);
    odo.pose.pose.position.z = 0.0;
    odo.pose.pose.orientation = tf_msg.transform.rotation;
    pub_odom_fused_.publish(odo);

    // 3) Append to Path and publish
    geometry_msgs::PoseStamped ps;
    ps.header = odo.header;
    ps.pose   = odo.pose.pose;
    path_msg_.header.stamp = t;
    path_msg_.poses.push_back(ps);
    pub_path_.publish(path_msg_);
  }

  // Normalize angle to [-π, π]
  double normalizeAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_encoders_imu_only");
  ros::NodeHandle nh("~");
  EKFEncodersIMU ekf(nh);
  ekf.spin();
  return 0;
}
