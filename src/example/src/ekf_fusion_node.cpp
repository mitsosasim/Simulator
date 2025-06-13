// =======================================================
// ekf_fusion_node.cpp
//
// 5-state EKF: [x, y, θ, v, ω] for Ackermann‐steering RC car.
// Fuses:
//   • /automobile/wheel_encoder/odometry  (v, ω)  @ ~100 Hz
//   • /automobile/IMU                     (yaw)  @ 10 Hz
//   • /sign_detector/sign_poses + sign_labels      @ ~20 Hz
//   • /ground_truth_path                      (one-shot init)
//
// Publishes:
//   • /ekf/odom   (Odometry of fused [x,y,θ] in “map” frame)
//   • /ekf/path   (nav_msgs/Path of fused trajectory)
//   • TF “map → chassis::link”
//
// Assumes wheel_encoder_plugin supplies correct Ackermann (v, ω).
// =======================================================

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <utils_ros/IMU.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>

// Custom message for sign labels
#include <utils_ros/SignLabelArray.h>

class EKFFusion
{
public:
  EKFFusion(ros::NodeHandle& nh)
    : nh_(nh),
      tf_listener_(tf_buffer_),
      state_(Eigen::Matrix<double,5,1>::Zero()),
      cov_(Eigen::Matrix<double,5,5>::Zero()),
      initialized_(false),
      last_odom_time_(ros::Time(0)),
      last_tf_stamp_(ros::Time(0))
  {
    // 1) Load hardcoded landmarks
    loadLandmarks();

    // 2) Initial covariance: moderate on x,y,θ; larger on v, ω
    cov_.setZero();
    cov_(0,0) = 0.01;   // var(x) = (0.1 m)^2
    cov_(1,1) = 0.01;   // var(y) = (0.1 m)^2
    cov_(2,2) = 0.05;   // var(θ) ≈ (12°)^2
    cov_(3,3) = 0.10;   // var(v)
    cov_(4,4) = 0.05;   // var(ω)

    // 3) Process noise Q
    Q_ = Eigen::Matrix<double,5,5>::Zero();
    Q_(3,3) = 2.5e-3;   // var of acceleration noise in v
    Q_(4,4) = 4e-4;     // var of yaw-accel noise in ω

    // 4) Measurement noise R
    R_odom_ = Eigen::Matrix2d::Zero();
    R_odom_(0,0) = 0.02 * 0.02;  // σ_v ≈ 0.02 m/s
    R_odom_(1,1) = 0.01 * 0.01;  // σ_ω ≈ 0.01 rad/s

    R_imu_ = 9e-4;               // σ_yaw ≈ 0.03 rad

    R_vis_ = Eigen::Matrix2d::Identity() * 1e-2;  // σ_pos ≈ 0.1 m

    // 5) ROS subscribers & publishers
    sub_odom_   = nh_.subscribe("/automobile/wheel_encoder/odometry",
                                1, &EKFFusion::odomCallback, this);
    sub_imu_    = nh_.subscribe("/automobile/IMU",
                                1, &EKFFusion::imuCallback, this);
    sub_vision_ = nh_.subscribe("/sign_detector/sign_poses",
                                1, &EKFFusion::visionCallback, this);
    sub_labels_ = nh_.subscribe("/sign_detector/sign_labels",
                                1, &EKFFusion::visionLabelCallback, this);
    sub_gt_path_= nh_.subscribe("/ground_truth_path",
                                1, &EKFFusion::groundTruthCallback, this);

    pub_odom_fused_ = nh_.advertise<nav_msgs::Odometry>("/ekf/odom", 1);
    pub_path_       = nh_.advertise<nav_msgs::Path>    ("/ekf/path", 1);

    path_msg_.header.frame_id = "map";

    ROS_INFO("[EKF] EKF Fusion node initialized.");
  }

  void spin() { ros::spin(); }

private:
  // ROS handles & TF
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_, sub_imu_, sub_vision_, sub_labels_, sub_gt_path_;
  ros::Publisher  pub_odom_fused_, pub_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer               tf_buffer_;
  tf2_ros::TransformListener    tf_listener_;

  // EKF state [x, y, θ, v, ω]
  Eigen::Matrix<double,5,1> state_;
  Eigen::Matrix<double,5,5> cov_;

  // Noise matrices
  Eigen::Matrix<double,5,5> Q_;
  Eigen::Matrix2d           R_odom_;
  double                    R_imu_;
  Eigen::Matrix2d           R_vis_;

  // Initialization & timing
  bool      initialized_;
  ros::Time last_odom_time_;
  ros::Time last_tf_stamp_;  // to suppress redundant TF

  // Landmarks in map frame
  struct Landmark { std::string cls; Eigen::Vector2d pos; };
  std::vector<Landmark> landmarks_;

  // Buffers for vision measurements
  std::vector<geometry_msgs::Pose> latest_poses_;
  std::vector<std::string>         latest_labels_;

  // Path message
  nav_msgs::Path path_msg_;

  // -------- CALLBACKS --------

  // 1) Ground truth callback for one-shot init
  void groundTruthCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    if (initialized_ || path_msg->poses.empty()) return;
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
    ROS_INFO("[EKF] Initialized from ground truth: x=%.3f y=%.3f θ=%.3f",
             state_(0), state_(1), state_(2));
    // No further ground-truth needed
    sub_gt_path_.shutdown();
  }

  // 2) Odometry callback: predict + measurement update (v, ω)
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    if (!initialized_) return;
    double v_raw = odom_msg->twist.twist.linear.x;
    double w_raw = odom_msg->twist.twist.angular.z;
    ros::Time t = odom_msg->header.stamp;
    double dt = last_odom_time_.isZero() ? 0.0 : (t - last_odom_time_).toSec();
    last_odom_time_ = t;
    if (dt > 0.0) {
      predict(dt);
      ROS_DEBUG("[EKF] Predict step dt=%.4f: state before predict x=%.3f y=%.3f θ=%.3f", dt, state_(0), state_(1), state_(2));
    }
    // Measurement update for v and ω
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
    ROS_DEBUG("[EKF] Odometry update: v=%.3f w=%.3f, innov=(%.3f,%.3f)", v_raw, w_raw, innov.x(), innov.y());
    publishFused(t);
  }

  // 3) IMU callback: yaw measurement update
  void imuCallback(const utils_ros::IMU::ConstPtr& imu_msg) {
    if (!initialized_) return;
    double meas_yaw = imu_msg->yaw;
    ros::Time t = ros::Time::now();
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
    ROS_DEBUG("[EKF] IMU update: yaw=%.3f, innov=%.3f", meas_yaw, innov);
    publishFused(t);
  }

  // 4) Vision pose callback: buffer poses in robot frame
  void visionCallback(const geometry_msgs::PoseArray::ConstPtr& poses_msg) {
    if (!initialized_) return;
    latest_poses_.clear();
    // Expect PoseArray poses already in chassis::link frame
    for (const auto& p : poses_msg->poses) {
      latest_poses_.push_back(p);
    }
    tryFuseVision();
  }

  // 5) Vision labels callback: buffer labels
  void visionLabelCallback(const utils_ros::SignLabelArray::ConstPtr& labels_msg) {
    latest_labels_ = labels_msg->labels;
    tryFuseVision();
  }

  // Check if we have matched poses & labels, then fuse
  void tryFuseVision() {
    if (latest_poses_.empty() || latest_poses_.size() != latest_labels_.size()) {
      ROS_DEBUG_THROTTLE(2.0, "[EKF] Vision: pose/label size mismatch or empty (poses=%zu labels=%zu).",
                         latest_poses_.size(), latest_labels_.size());
      return;
    }
    ROS_INFO("[EKF] Vision: Fusing %zu detections.", latest_poses_.size());
    // Build 2D measurement vectors (robot frame)
    std::vector<Eigen::Vector2d> meas;
    meas.reserve(latest_poses_.size());
    for (const auto& p : latest_poses_) {
      meas.emplace_back(p.position.x, p.position.y);
      ROS_DEBUG_STREAM("[EKF] Vision raw m (robot frame) = (" << p.position.x << ", " << p.position.y << ")");
    }
    // Use current time for fusion timestamp
    ros::Time t = ros::Time::now();
    updateVision(meas, latest_labels_, t);
    latest_poses_.clear();
    latest_labels_.clear();
  }

void updateVision(const std::vector<Eigen::Vector2d>& meas,
                  const std::vector<std::string>& labels,
                  const ros::Time& t)
{
  for (size_t i = 0; i < meas.size(); ++i) {
    const auto& m = meas[i];          // detection in robot (chassis) frame
    const auto& cls = labels[i];      // class string
    ROS_DEBUG_STREAM("[EKF] Vision: class=" << cls << " measurement m=" << m.transpose());

    // Transform m into map frame using current state
    double th = state_(2);
    double cos_th = std::cos(th), sin_th = std::sin(th);
    Eigen::Vector2d robot_pos(state_(0), state_(1));
    Eigen::Vector2d m_map;
    m_map.x() = robot_pos.x() + cos_th * m.x() - sin_th * m.y();
    m_map.y() = robot_pos.y() + sin_th * m.x() + cos_th * m.y();
    ROS_DEBUG_STREAM("[EKF] Vision: m_map = " << m_map.transpose());

    // Mahalanobis gating for association
    double best_maha = std::numeric_limits<double>::infinity();
    Eigen::Vector2d L;
    Eigen::Matrix2d best_S = Eigen::Matrix2d::Zero();
    Eigen::Vector2d best_innov;

    for (const auto& lm : landmarks_) {
      if (lm.cls != cls) continue;
      Eigen::Vector2d innov_map = m_map - lm.pos;

      // Innovation covariance S = R_vis_ (since measurement in map frame, no state uncertainty)
      // If you want to include state covariance, transform cov_ to map frame and add here.
      Eigen::Matrix2d S = R_vis_;

      double maha = innov_map.transpose() * S.inverse() * innov_map;
      ROS_DEBUG_STREAM("[EKF] Mahalanobis d^2 to lm " << lm.pos.transpose() << " = " << maha);

      if (maha < best_maha) {
        best_maha = maha;
        L = lm.pos;
        best_S = S;
        best_innov = innov_map;
      }
    }

    // Mahalanobis threshold for 2D, 99% confidence: ~9.21
    const double MAHA_THRESHOLD = 9.21;
    if (best_maha > MAHA_THRESHOLD) {
      ROS_WARN_STREAM("[EKF] Vision: No landmark match for class " << cls
                      << " at global " << m_map.transpose()
                      << " (closest Mahalanobis d^2 " << best_maha << ")");
      continue;
    }
    ROS_INFO_STREAM("[EKF] Vision: Matched class " << cls
                    << " global=" << m_map.transpose()
                    << " to landmark " << L.transpose()
                    << " (Mahalanobis d^2=" << best_maha << ")");

    // Predicted measurement in robot frame: p_hat = R(-θ)*(L - robot_pos)
    Eigen::Vector2d delta = L - robot_pos;
    Eigen::Vector2d p_hat;
    p_hat.x() =  cos_th * delta.x() + sin_th * delta.y();
    p_hat.y() = -sin_th * delta.x() + cos_th * delta.y();
    Eigen::Vector2d innov = m - p_hat;
    ROS_DEBUG_STREAM("[EKF] Vision: innov (robot frame) = " << innov.transpose());

    // Build Jacobian H_vis (2x5)
    Eigen::Matrix<double,2,5> H_vis = Eigen::Matrix<double,2,5>::Zero();
    H_vis(0,0) = -cos_th;  H_vis(0,1) = -sin_th;
    H_vis(1,0) =  sin_th;  H_vis(1,1) = -cos_th;
    double dx = L.x() - robot_pos.x();
    double dy = L.y() - robot_pos.y();
    H_vis(0,2) = dx * (-sin_th) + dy * (-cos_th);
    H_vis(1,2) = dx * ( cos_th) + dy * (-sin_th);

    Eigen::Matrix2d S = H_vis * cov_ * H_vis.transpose() + R_vis_;
    Eigen::Matrix<double,5,2> K = cov_ * H_vis.transpose() * S.inverse();

    ROS_INFO_STREAM("[EKF] Vision: Innovation = " << innov.transpose());
    ROS_INFO_STREAM("[EKF] Vision: Kalman gain (first row) = " << K.row(0));
    ROS_INFO_STREAM("[EKF] Vision: m (measured) = " << m.transpose());
    ROS_INFO_STREAM("[EKF] Vision: L (landmark) = " << L.transpose());
    ROS_INFO_STREAM("[EKF] Vision: p_hat (predicted) = " << p_hat.transpose());

    // State update
    state_ += K * innov;
    state_(2) = normalizeAngle(state_(2));
    cov_ = (Eigen::Matrix<double,5,5>::Identity() - K * H_vis) * cov_;
    ROS_INFO_STREAM("[EKF] Vision: Updated state x=" << state_(0) << " y=" << state_(1) << " θ=" << state_(2));
  }
  publishFused(t);

  // -------- EKF PREDICTION --------
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
    // v, w unchanged (process noise will affect covariance)

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

  // -------- PUBLISH RESULTS --------
  void publishFused(const ros::Time& t) {
    // Suppress redundant TF if same timestamp
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

  // Load hardcoded landmarks
  void loadLandmarks() {
    // Crosswalk
    landmarks_.push_back({"crosswalk", {7.312, -3.240}});
    landmarks_.push_back({"crosswalk", {6.627, -4.181}});
    landmarks_.push_back({"crosswalk", {6.777, -1.456}});
    landmarks_.push_back({"crosswalk", {6.130, -2.342}});
    landmarks_.push_back({"crosswalk", {1.109, -12.486}});
    landmarks_.push_back({"crosswalk", {0.195, -11.869}});
    // Enter‐highway
    landmarks_.push_back({"enterhighway", {5.871, -13.970}});
    landmarks_.push_back({"enterhighway", {9.412, -4.850}});
    // Leave‐highway
    landmarks_.push_back({"leavehighway", {10.686, -6.276}});
    landmarks_.push_back({"leavehighway", {6.520, -12.700}});
    landmarks_.push_back({"leavehighway", {8.458, -14.359}});
    // Oneway
    landmarks_.push_back({"oneway", {3.341, -9.821}});
    landmarks_.push_back({"oneway", {2.406, -9.821}});
    landmarks_.push_back({"oneway", {9.196, -14.375}});
    // Parking
    landmarks_.push_back({"parking", {4.047, -2.358}});
    landmarks_.push_back({"parking", {2.857, -2.358}});
    landmarks_.push_back({"parking", {2.779, -1.452}});
    landmarks_.push_back({"parking", {4.459, -1.452}});
    // Priority
    landmarks_.push_back({"priority", {3.675, -13.050}});
    landmarks_.push_back({"priority", {0.204, -6.031}});
    landmarks_.push_back({"priority", {5.544, -11.381}});
    landmarks_.push_back({"priority", {4.589, -5.997}});
    // Prohibited
    landmarks_.push_back({"prohibited", {3.323, -7.581}});
    landmarks_.push_back({"prohibited", {2.387, -7.581}});
    landmarks_.push_back({"prohibited", {11.578, -4.504}});
    // Roundabout
    landmarks_.push_back({"roundabout", {8.766, -4.179}});
    landmarks_.push_back({"roundabout", {10.320, -4.869}});
    landmarks_.push_back({"roundabout", {11.015, -3.297}});
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_fusion_node");
  ros::NodeHandle nh("~");
  EKFFusion ekf(nh);
  ekf.spin();
  return 0;
}
