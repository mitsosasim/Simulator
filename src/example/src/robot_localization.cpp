// =======================================================
// ekf_encoders_imu_only.cpp
//
// 5-state EKF: [x, y, θ, v, ω] for Ackermann‐steering RC car.
// Fuses:
//   • /automobile/wheel_encoder/odometry  (v, ω)  @ ~100 Hz
//   • /automobile/IMU                     (yaw)   @ 10 Hz
//   • /ground_truth_path                  (one-shot init)
//
// Publishes:
//   • /ekf/odom   (Odometry of fused [x,y,θ] in “map” frame)
//   • /ekf/path   (nav_msgs/Path of fused trajectory)
//   • TF “map → chassis::link”
// =======================================================

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <utils_ros/IMU.h>
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
      last_odom_time_(ros::Time(0))
  {
    // --- 1) Set initial covariance ---
    cov_.setZero();
    cov_(0,0) = 0.01;   // var(x)  = (0.1 m)²
    cov_(1,1) = 0.01;   // var(y)  = (0.1 m)²
    cov_(2,2) = 0.05;   // var(θ)  = (0.22 rad)² ≈ (12°)²
    cov_(3,3) = 0.10;   // var(v)  = (0.316 m/s)²
    cov_(4,4) = 0.05;   // var(ω)  = (0.223 rad/s)²

    // --- 2) Process noise Q (5×5) ---
    Q_ = Eigen::Matrix<double,5,5>::Zero();
    Q_(3,3) = 2.5e-3; // v acceleration noise
    Q_(4,4) = 4e-4;   // ω acceleration noise

    // --- 3) Measurement noise ---
    // Odometry: encoder noise: σ_v ≈ 0.02 m/s, σ_ω ≈ 0.01 rad/s
    R_odom_ = Eigen::Matrix2d::Zero();
    R_odom_(0,0) = 0.02 * 0.02;
    R_odom_(1,1) = 0.01 * 0.01;
    // IMU yaw: σ_yaw ≈ 0.03 rad
    R_imu_ = 9e-4;

    // --- 4) ROS subscribers & publishers ---
    sub_odom_   = nh_.subscribe("/automobile/wheel_encoder/odometry",
                                1, &EKFEncodersIMU::odomCallback, this);
    sub_imu_    = nh_.subscribe("/automobile/IMU",
                                1, &EKFEncodersIMU::imuCallback, this);
    sub_gt_path_= nh_.subscribe("/ground_truth_path",
                                1, &EKFEncodersIMU::groundTruthCallback, this);

    pub_odom_fused_ = nh_.advertise<nav_msgs::Odometry>("/robot_localization/odom", 1);
    pub_path_       = nh_.advertise<nav_msgs::Path>    ("/robot_localization/path", 1);

    path_msg_.header.frame_id = "map";
  }

  void spin() { ros::spin(); }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_, sub_imu_, sub_gt_path_;
  ros::Publisher  pub_odom_fused_, pub_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Time last_tf_stamp_;

  Eigen::Matrix<double,5,1>   state_;
  Eigen::Matrix<double,5,5>   cov_;
  Eigen::Matrix<double,5,5>   Q_;       // Process noise (5×5)
  Eigen::Matrix2d             R_odom_;  // Odometry measurement noise (2×2)
  double                      R_imu_;   // IMU yaw noise (scalar)

  bool            initialized_;
  ros::Time       last_odom_time_;
  nav_msgs::Path  path_msg_;

  // 1) Ground-truth callback: initialize EKF using first message
  void groundTruthCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    if (initialized_ || path_msg->poses.empty()) return;

    // Take the first pose in the path as initial state
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

    // Initialize v and ω to zero
    state_(3) = 0.0;
    state_(4) = 0.0;

    last_odom_time_ = path_msg->poses.front().header.stamp;
    initialized_ = true;
    ROS_INFO("EKF (encoders+IMU) initialized: x=%.3f y=%.3f θ=%.3f",
             state_(0), state_(1), state_(2));

    sub_gt_path_.shutdown();  // No longer need ground-truth
  }

  // 2) Odometry callback: predict + odom measurement update
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    if (!initialized_) return;

    // 2a) Extract raw (v, ω) from wheel encoder plugin
    double v_raw = odom_msg->twist.twist.linear.x;
    double w_raw = odom_msg->twist.twist.angular.z;
    ros::Time t = odom_msg->header.stamp;

    // 2b) Compute dt and run predict step using internal (v, ω)
    double dt = last_odom_time_.isZero() ? 0.0 : (t - last_odom_time_).toSec();
    last_odom_time_ = t;
    if (dt > 0.0) predict(dt);

    // 2c) Build measurement update for v, ω
    Eigen::Matrix<double,2,5> H_odom = Eigen::Matrix<double,2,5>::Zero();
    H_odom(0,3) = 1.0;  // measurement v = state_[3]
    H_odom(1,4) = 1.0;  // measurement ω = state_[4]

    Eigen::Vector2d z_odom(v_raw, w_raw);
    Eigen::Vector2d z_hat = H_odom * state_;
    Eigen::Vector2d y = z_odom - z_hat;                 // Innovation

    Eigen::Matrix2d S = H_odom * cov_ * H_odom.transpose() + R_odom_;
    Eigen::Matrix<double,5,2> K = cov_ * H_odom.transpose() * S.inverse();
    state_ += K * y;
    cov_   = (Eigen::Matrix<double,5,5>::Identity() - K * H_odom) * cov_;

    publishFused(t);
  }

  // 3) IMU callback: yaw measurement update
  void imuCallback(const utils_ros::IMU::ConstPtr& imu_msg) {
    if (!initialized_) return;

    double meas_yaw = imu_msg->yaw;
    ros::Time t = ros::Time::now(); // or imu_msg->header.stamp if available

    // Build H_imu (1×5) that picks out θ
    Eigen::Matrix<double,1,5> H_imu = Eigen::Matrix<double,1,5>::Zero();
    H_imu(0,2) = 1.0;

    double z = meas_yaw;
    double z_hat = state_(2);
    double innov = normalizeAngle(z - z_hat);

    double S = (H_imu * cov_ * H_imu.transpose())(0,0) + R_imu_;
    Eigen::Matrix<double,5,1> K = cov_ * H_imu.transpose() / S;

    state_ += K * innov;
    state_(2) = normalizeAngle(state_(2)); // Normalize yaw
    cov_ = (Eigen::Matrix<double,5,5>::Identity() - K * H_imu) * cov_;

    publishFused(t);
  }

  // EKF PREDICTION
  void predict(double dt) {
    double x  = state_(0);
    double y  = state_(1);
    double th = state_(2);
    double v  = state_(3);
    double w  = state_(4);

    // 1) Nonlinear state propagation
    state_(0) += v * cos(th) * dt;
    state_(1) += v * sin(th) * dt;
    state_(2) += w * dt;
    // v and w remain same (constant) – process noise will nudge them

    // 2) Build Jacobian F (5×5)
    Eigen::Matrix<double,5,5> F = Eigen::Matrix<double,5,5>::Identity();
    F(0,2) = -v * sin(th) * dt;   // ∂x/∂θ
    F(0,3) =  cos(th) * dt;       // ∂x/∂v
    F(1,2) =  v * cos(th) * dt;   // ∂y/∂θ
    F(1,3) =  sin(th) * dt;       // ∂y/∂v
    F(2,4) =  dt;                 // ∂θ/∂ω

    cov_ = F * cov_ * F.transpose() + Q_;
  }

  void publishFused(const ros::Time& t) {
    // Prevent duplicate TF with same timestamp
    if (t == last_tf_stamp_) return;
    last_tf_stamp_ = t;
    // 1) Broadcast TF “map → chassis::link”
    geometry_msgs::TransformStamped tf;
    tf.header.stamp    = t;
    tf.header.frame_id = "map";
    tf.child_frame_id  = "chassis::link";
    tf.transform.translation.x = state_(0);
    tf.transform.translation.y = state_(1);
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state_(2));
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(tf);

    // 2) Publish Odometry
    nav_msgs::Odometry odo;
    odo.header.stamp    = t;
    odo.header.frame_id = "map";
    odo.child_frame_id  = "chassis::link";
    odo.pose.pose.position.x = state_(0);
    odo.pose.pose.position.y = state_(1);
    odo.pose.pose.position.z = 0.0;
    odo.pose.pose.orientation = tf.transform.rotation;
    pub_odom_fused_.publish(odo);

    // 3) Append to Path and publish
    geometry_msgs::PoseStamped ps;
    ps.header = odo.header;
    ps.pose   = odo.pose.pose;
    path_msg_.header.stamp = t;
    path_msg_.poses.push_back(ps);
    pub_path_.publish(path_msg_);
  }

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


