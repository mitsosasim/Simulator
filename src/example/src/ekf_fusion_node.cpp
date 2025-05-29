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

// Custom message for labels
#include <utils_ros/SignLabelArray.h>

class EKFFusion
{
public:
  EKFFusion(ros::NodeHandle& nh)
    : nh_(nh),
      tf_listener_(tf_buffer_),
      state_(Eigen::Vector3d::Zero()),
      cov_(Eigen::Matrix3d::Identity() * 1e-3),
      last_odom_time_(ros::Time(0)),
      initialized_(false)
  {
    loadLandmarks();

    // Noise matrices
    Q_      = Eigen::Matrix2d::Identity() * 1e-4;
    R_imu_  = 1e-2;
    R_vis_  = Eigen::Matrix2d::Identity() * 1e-1;

    // Subscribers
    sub_odom_   = nh_.subscribe("/automobile/wheel_encoder/odometry",  1, &EKFFusion::odomCallback,      this);
    sub_imu_    = nh_.subscribe("/automobile/IMU",                     1, &EKFFusion::imuCallback,       this);
    sub_vision_ = nh_.subscribe("/sign_detector/sign_poses",           1, &EKFFusion::visionCallback,    this);
    sub_labels_ = nh_.subscribe("/sign_detector/sign_labels",          1, &EKFFusion::visionLabelCallback,this);
    sub_gt_path_= nh_.subscribe("/ground_truth_path",                  1, &EKFFusion::groundTruthCallback,this);

    // Publishers
    pub_odom_fused_ = nh_.advertise<nav_msgs::Odometry>("/ekf/odom", 1);
    pub_path_       = nh_.advertise<nav_msgs::Path>    ("/ekf/path", 1);

    path_msg_.header.frame_id = "map";
  }

  void spin() { ros::spin(); }

private:
  // ROS handles
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_, sub_imu_, sub_vision_, sub_labels_, sub_gt_path_;
  ros::Publisher  pub_odom_fused_, pub_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer               tf_buffer_;
  tf2_ros::TransformListener    tf_listener_;

  // Buffers for vision pairing
  std::vector<geometry_msgs::Pose> latest_poses_;
  std::vector<std::string>         latest_labels_;
  bool initialized_;
  ros::Time last_odom_time_;

  // EKF state & covariance
  Eigen::Vector3d state_;
  Eigen::Matrix3d cov_;

  // Noise parameters
  Eigen::Matrix2d Q_;
  double          R_imu_;
  Eigen::Matrix2d R_vis_;

  // Landmarks: class tag + (x,y)
  struct Landmark { std::string cls; Eigen::Vector2d pos; };
  std::vector<Landmark> landmarks_;

  nav_msgs::Path path_msg_;

  // ====================
  //  CALLBACKS
  // ====================

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    if (!initialized_) return;
    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;
    ros::Time t = odom_msg->header.stamp;
    double dt = last_odom_time_.isZero() ? 0.0 : (t - last_odom_time_).toSec();
    last_odom_time_ = t;
    if (dt > 0.0) predict(v, w, dt);
    publishFused(t);
  }

  void imuCallback(const utils_ros::IMU::ConstPtr& imu_msg) {
    if (!initialized_) return;
    updateIMU(imu_msg->yaw, ros::Time::now());
  }

  void visionCallback(const geometry_msgs::PoseArray::ConstPtr& poses_msg) {
    if (!initialized_) return;
    latest_poses_.clear();
    geometry_msgs::PoseStamped in, out;
    in.header = poses_msg->header;
    in.header.frame_id = "camera::link_camera";
    for (const auto& p : poses_msg->poses) {
      in.pose = p;
      tf_buffer_.transform(in, out, "chassis::link");
      latest_poses_.push_back(out.pose);
    }
    tryFuseVision();
  }

  void visionLabelCallback(const utils_ros::SignLabelArray::ConstPtr& labels_msg) {
    latest_labels_ = labels_msg->labels;
    tryFuseVision();
  }

  void groundTruthCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    if (initialized_ || path_msg->poses.empty()) return;
    const auto& p0 = path_msg->poses.front().pose;
    state_(0) = p0.position.x;
    state_(1) = p0.position.y;
    tf2::Quaternion q(
      p0.orientation.x, p0.orientation.y,
      p0.orientation.z, p0.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    state_(2) = yaw;
    last_odom_time_ = path_msg->poses.front().header.stamp;
    initialized_ = true;
    ROS_INFO("EKF initialized: x=%.3f y=%.3f yaw=%.3f",
             state_(0), state_(1), state_(2));
    sub_gt_path_.shutdown();
  }

  // ====================
  //  EKF STEPS
  // ====================

  void predict(double v, double w, double dt) {
    double th = state_(2);
    state_(0) += v * cos(th) * dt;
    state_(1) += v * sin(th) * dt;
    state_(2) += w * dt;

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -v * sin(th) * dt;
    F(1,2) =  v * cos(th) * dt;

    Eigen::Matrix<double,3,2> B;
    B << cos(th)*dt, 0,
         sin(th)*dt, 0,
         0,          dt;

    cov_ = F * cov_ * F.transpose() + B * Q_ * B.transpose();
  }

  void updateIMU(double meas_yaw, const ros::Time& t) {
    double z = meas_yaw;
    double y = normalizeAngle(z - state_(2));
    Eigen::Vector3d H(0, 0, 1);
    double S = (H.transpose() * cov_ * H)(0) + R_imu_;
    Eigen::Vector3d K = cov_ * H / S;

    state_ += K * y;
    state_(2) = normalizeAngle(state_(2));
    cov_ = (Eigen::Matrix3d::Identity() - K * H.transpose()) * cov_;

    publishFused(t);
  }

  void tryFuseVision() {
    if (latest_poses_.size() != latest_labels_.size() ||
        latest_poses_.empty())
      return;

    // Build measurements
    std::vector<Eigen::Vector2d> meas;
    meas.reserve(latest_poses_.size());
    for (auto& p : latest_poses_)
      meas.emplace_back(p.position.x, p.position.y);

    updateVision(meas, latest_labels_, ros::Time::now());

    latest_poses_.clear();
    latest_labels_.clear();
  }

  void updateVision(const std::vector<Eigen::Vector2d>& meas,
                    const std::vector<std::string>& labels,
                    const ros::Time& t)
  {
    for (size_t i = 0; i < meas.size(); ++i) {
      const auto& m   = meas[i];
      const auto& cls = labels[i];

      // Find nearest landmark of same class
      double best_d = 1e9;
      Eigen::Vector2d L;
      for (auto& lm : landmarks_) {
        if (lm.cls != cls) continue;
        double d = (lm.pos - m).norm();
        if (d < best_d) {
          best_d = d;
          L = lm.pos;
        }
      }
      if (best_d > 5.0) continue;  // too far

      double th = state_(2);
      Eigen::Rotation2Dd R(-th);
      Eigen::Vector2d p_hat = R * (L - state_.head<2>());
      Eigen::Vector2d y     = m - p_hat;

      Eigen::Matrix<double,2,3> H;
      H << -cos(th), -sin(th),
            sin(th), -cos(th),
            (L.x()*sin(th) - L.y()*cos(th)
             + state_(0)*sin(th) - state_(1)*cos(th)),
           -(L.x()*cos(th) + L.y()*sin(th)
             - state_(0)*cos(th) - state_(1)*sin(th));

      Eigen::Matrix2d S = H * cov_ * H.transpose() + R_vis_;
      Eigen::Matrix<double,3,2> K = cov_ * H.transpose() * S.inverse();

      state_ += K * y;
      state_(2) = normalizeAngle(state_(2));
      cov_ = (Eigen::Matrix3d::Identity() - K * H) * cov_;
    }
    publishFused(t);
  }

  // ====================
  //  PUBLISH RESULTS
  // ====================

  void publishFused(const ros::Time& t) {
    // 1) TF
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

    // 2) Odometry
    nav_msgs::Odometry odo;
    odo.header.stamp    = t;
    odo.header.frame_id = "map";
    odo.child_frame_id  = "chassis::link";
    odo.pose.pose.position.x = state_(0);
    odo.pose.pose.position.y = state_(1);
    odo.pose.pose.position.z = 0.0;
    odo.pose.pose.orientation = tf.transform.rotation;
    pub_odom_fused_.publish(odo);

    // 3) Path
    geometry_msgs::PoseStamped ps;
    ps.header = odo.header;
    ps.pose   = odo.pose.pose;
    path_msg_.poses.push_back(ps);
    pub_path_.publish(path_msg_);
  }

  // ====================
  //  UTILITIES
  // ====================

  double normalizeAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void loadLandmarks() {
    // (copy all your landmark pushes here, unchanged)
    landmarks_.push_back({"crosswalk",    {7.312, -3.240}});
    landmarks_.push_back({"crosswalk",    {6.627, -4.181}});
    // … etc …
    landmarks_.push_back({"roundabout",   {11.015, -3.297}});
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_fusion_node");
  ros::NodeHandle nh("~");
  EKFFusion ekf(nh);
  ekf.spin();
  return 0;
}
