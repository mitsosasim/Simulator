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
// Assumes wheel_encoder_plugin now supplies correct Ackermann (v, ω).
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
      // Initialize 5×1 state: [x, y, θ, v, ω]ᵀ
      state_(Eigen::Matrix<double,5,1>::Zero()),
      // Initialize 5×5 covariance; small initial certainty on x,y,θ, v, ω
      cov_(Eigen::Matrix<double,5,5>::Zero()),
      initialized_(false),
      last_odom_time_(ros::Time(0))
  {
    // --- 1) Load landmarks from map (hardcoded) ---
    loadLandmarks();

    // --- 2) Set initial covariance ---
    // We assume moderate uncertainty in initial pose and larger in velocity.
    cov_.setZero();
    cov_(0,0) = 0.01;   // var(x)  = (0.1 m)²
    cov_(1,1) = 0.01;   // var(y)  = (0.1 m)²
    cov_(2,2) = 0.05;   // var(θ)  = (0.22 rad)² ≈ (12°)²
    cov_(3,3) = 0.10;   // var(v)  = (0.316 m/s)²
    cov_(4,4) = 0.05;   // var(ω)  = (0.223 rad/s)²

    // --- 3) Process noise Q (5×5) ---
    // We model small acceleration noise in v, and yaw‐acceleration noise in ω.
    Q_ = Eigen::Matrix<double,5,5>::Zero();
    // σ_acc_v ≈ 0.05 m/s²  ⇒ var ≈ (0.05)² = 2.5e-3
    Q_(3,3) = 2.5e-3;
    // σ_acc_ω ≈ 0.02 rad/s²  ⇒ var ≈ (0.02)² = 4e-4
    Q_(4,4) = 4e-4;

    // --- 4) Measurement noise R for odometry (2×2), IMU yaw, and vision (2×2) ---
    // Odometry: encoder noise: σ_v ≈ 0.02 m/s, σ_ω ≈ 0.01 rad/s
    R_odom_ = Eigen::Matrix2d::Zero();
    R_odom_(0,0) = 0.02 * 0.02;
    R_odom_(1,1) = 0.01 * 0.01;

    // IMU yaw: σ_yaw ≈ 0.03 rad  ⇒ var ≈ (0.03)² = 9e-4
    R_imu_ = 9e-4;

    // Vision landmark: σ_pos ≈ 0.1 m  ⇒ var ≈ (0.1)² = 1e-2
    R_vis_ = Eigen::Matrix2d::Identity() * 1e-2;

    // --- 5) ROS subscribers & publishers ---
    sub_odom_   = nh_.subscribe("/automobile/wheel_encoder/odometry",
                                1, &EKFFusion::odomCallback,    this);
    sub_imu_    = nh_.subscribe("/automobile/IMU",
                                1, &EKFFusion::imuCallback,     this);
    sub_vision_ = nh_.subscribe("/sign_detector/sign_poses",
                                1, &EKFFusion::visionCallback,  this);
    sub_labels_ = nh_.subscribe("/sign_detector/sign_labels",
                                1, &EKFFusion::visionLabelCallback, this);
    sub_gt_path_= nh_.subscribe("/ground_truth_path",
                                1, &EKFFusion::groundTruthCallback, this);

    pub_odom_fused_ = nh_.advertise<nav_msgs::Odometry>("/ekf/odom", 1);
    pub_path_       = nh_.advertise<nav_msgs::Path>    ("/ekf/path", 1);

    path_msg_.header.frame_id = "map";
  }

  void spin() { ros::spin(); }

private:
  // --------------------------------
  //  ROS handles & TF
  // --------------------------------
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_, sub_imu_, sub_vision_, sub_labels_, sub_gt_path_;
  ros::Publisher  pub_odom_fused_, pub_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer               tf_buffer_;
  tf2_ros::TransformListener    tf_listener_;

  // --------------------------------
  //  EKF state (5×1) and covariance (5×5)
  //    [ x,    y,    θ,    v,    ω ]ᵀ
  // --------------------------------
  Eigen::Matrix<double,5,1>   state_;
  Eigen::Matrix<double,5,5>   cov_;

  // --------------------------------
  //  Noise matrices
  // --------------------------------
  Eigen::Matrix<double,5,5> Q_;       // Process noise (5×5)
  Eigen::Matrix2d           R_odom_;  // Odometry measurement noise (2×2)
  double                    R_imu_;   // IMU yaw noise (scalar)
  Eigen::Matrix2d           R_vis_;   // Vision (landmark) noise (2×2)

  // --------------------------------
  //  Buffers & initialization flag
  // --------------------------------
  bool            initialized_;        // Set true after ground-truth init
  ros::Time       last_odom_time_;     // Timestamp of last odometry

  // --------------------------------
  //  Landmark storage
  // --------------------------------
  struct Landmark { std::string cls; Eigen::Vector2d pos; };
  std::vector<Landmark>       landmarks_;

  // --------------------------------
  //  Buffers for vision pairing
  // --------------------------------
  std::vector<geometry_msgs::Pose> latest_poses_;
  std::vector<std::string>         latest_labels_;

  // --------------------------------
  //  Path message for publishing trajectory
  // --------------------------------
  nav_msgs::Path path_msg_;

  // ====================
  //  CALLBACKS
  // ====================

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

    // Initialize v and ω to zero (or small)
    state_(3) = 0.0;
    state_(4) = 0.0;

    last_odom_time_ = path_msg->poses.front().header.stamp;
    initialized_ = true;
    ROS_INFO("EKF initialized: x=%.3f y=%.3f θ=%.3f",
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
    // H_odom (2×5): picks out v and ω from state: [ x,y,θ,v,ω ]
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

    // 2d) Publish fused result after odometry update
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

  // 4) Vision pose callback: buffer measured PoseArray → robot frame
  void visionCallback(const geometry_msgs::PoseArray::ConstPtr& poses_msg) {
    if (!initialized_) return;

    latest_poses_.clear();
    geometry_msgs::PoseStamped in, out;
    in.header = poses_msg->header;
    in.header.frame_id = "camera::link_camera";

    // Transform each detected sign pose to chassis::link frame
    for (const auto& p : poses_msg->poses) {
      in.pose = p;
      tf_buffer_.transform(in, out, "chassis::link");
      latest_poses_.push_back(out.pose);
    }

    tryFuseVision();
  }

  // 5) Vision labels callback: buffer labels
  void visionLabelCallback(const utils_ros::SignLabelArray::ConstPtr& labels_msg) {
    latest_labels_ = labels_msg->labels;
    tryFuseVision();
  }

  // Attempt to fuse vision if both poses & labels are available
  void tryFuseVision() {
    if (latest_poses_.size() != latest_labels_.size() || latest_poses_.empty()) {
      ROS_WARN_STREAM_THROTTLE(2.0, "[EKF] Vision: pose/label size mismatch or empty. poses=" << latest_poses_.size() << " labels=" << latest_labels_.size());
      return;
    }
    ROS_INFO_STREAM("[EKF] Vision: Fusing " << latest_poses_.size() << " detections.");

    // Build 2D measurement vector
    std::vector<Eigen::Vector2d> meas;
    meas.reserve(latest_poses_.size());
    for (const auto& p : latest_poses_)
      meas.emplace_back(p.position.x, p.position.y);

    updateVision(meas, latest_labels_, ros::Time::now());

    latest_poses_.clear();
    latest_labels_.clear();
  }

  // 6) VK: Vision update step (landmark association & EKF correction)
  void updateVision(const std::vector<Eigen::Vector2d>& meas,
                  const std::vector<std::string>& labels,
                  const ros::Time& t)
{
    for (size_t i = 0; i < meas.size(); ++i) {
        const auto& m   = meas[i];
        const auto& cls = labels[i];

        // Associate with nearest landmark of matching class
        double best_d = 1e9;
        Eigen::Vector2d L;
        for (const auto& lm : landmarks_) {
            if (lm.cls != cls) continue;
            double d = (lm.pos - m).norm();
            if (d < best_d) {
                best_d = d;
                L = lm.pos;
            }
        }
        if (best_d > 5.0) {
            ROS_WARN_STREAM_THROTTLE(2.0, "[EKF] Vision: No landmark match for class " << cls << " at " << m.transpose() << " (closest " << best_d << " m)");
            continue;
        }

        ROS_INFO_STREAM("[EKF] Vision: Updating with class " << cls << " at " << m.transpose() << " matched to landmark " << L.transpose() << " (dist " << best_d << " m)");

        // Compute predicted measurement: transform landmark into robot frame
        double th = state_(2);
        Eigen::Rotation2Dd R(-th);
        Eigen::Vector2d p_hat = R * (L - state_.head<2>());

        Eigen::Vector2d innov = m - p_hat; // Innovation in chassis frame

        // Build H_vis (2×5) Jacobian with respect to [x,y,θ,v,ω]
        Eigen::Matrix<double,2,5> H_vis = Eigen::Matrix<double,2,5>::Zero();
        double lx = L.x(), ly = L.y();
        double rx = state_(0), ry = state_(1);
        double dx = lx - rx, dy = ly - ry;

        H_vis(0,0) = -cos(th);
        H_vis(0,1) = -sin(th);
        H_vis(0,2) =  dx * (-sin(th)) + dy * (-cos(th));
        H_vis(1,0) =  sin(th);
        H_vis(1,1) = -cos(th);
        H_vis(1,2) =  dx * ( cos(th)) + dy * (-sin(th));

        Eigen::Matrix2d S = H_vis * cov_ * H_vis.transpose() + R_vis_;
        Eigen::Matrix<double,5,2> K = cov_ * H_vis.transpose() * S.inverse();

        ROS_INFO_STREAM("[EKF] Vision: Innovation = " << innov.transpose());
        ROS_INFO_STREAM("[EKF] Vision: Kalman gain (first row) = " << K.row(0));

        // State update
        state_ += K * innov;
        state_(2) = normalizeAngle(state_(2));
        cov_ = (Eigen::Matrix<double,5,5>::Identity() - K * H_vis) * cov_;
    }

    publishFused(t);
}

  // ====================
  //  EKF PREDICTION
  // ====================
  // Constant‐velocity/turn‐rate motion model
  void predict(double dt) {
    double x  = state_(0);
    double y  = state_(1);
    double th = state_(2);
    double v  = state_(3);
    double w  = state_(4);

    // 1) Nonlinear state propagation
    state_(0) += v * cos(th) * dt;
    state_(1) += v * sin(th) * dt;
    state_(2) += w * dt;          // θₖ₊₁ = θₖ + wₖ * dt
    // v and w remain same (constant) – process noise will nudge them

    // 2) Build Jacobian F (5×5)
    Eigen::Matrix<double,5,5> F = Eigen::Matrix<double,5,5>::Identity();
    F(0,2) = -v * sin(th) * dt;   // ∂x/∂θ
    F(0,3) =  cos(th) * dt;       // ∂x/∂v
    F(1,2) =  v * cos(th) * dt;   // ∂y/∂θ
    F(1,3) =  sin(th) * dt;       // ∂y/∂v
    F(2,4) =  dt;                 // ∂θ/∂ω
    // F(3,3)=1, F(4,4)=1 by Identity()

    // 3) Covariance propagation: P = F P Fᵀ + Q
    cov_ = F * cov_ * F.transpose() + Q_;
  }

  // ====================
  //  PUBLISH RESULTS
  // ====================
  void publishFused(const ros::Time& t) {
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
    path_msg_.header.stamp = t;  // or = odo.header.stamp

    path_msg_.poses.push_back(ps);
    pub_path_.publish(path_msg_);
  }

  // ====================
  //  UTILITY FUNCTIONS
  // ====================
  double normalizeAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // Load hardcoded map landmarks
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
