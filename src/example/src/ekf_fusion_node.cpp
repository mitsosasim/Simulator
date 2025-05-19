#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <utils/IMU.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>




class EKFFusion
{
public:
  EKFFusion(ros::NodeHandle& nh)
    : nh_(nh),
      state_(Eigen::Vector3d::Zero()),
      cov_(Eigen::Matrix3d::Identity() * 1e-3),
      last_odom_time_(ros::Time(0))   // <— add this
  {
    // Load landmarks from parameter server or hardcode
    loadLandmarks();

    // Noise covariance (tunable)
    Q_ = Eigen::Matrix2d::Identity() * 1e-4;      // process noise for [v, ω]
    R_imu_ = 1e-2;                                // variance for yaw
    R_vis_ = Eigen::Matrix2d::Identity() * 1e-1;  // variance for [x_cam, y_cam]

    // Subscribers
    sub_odom_   = nh_.subscribe("/automobile/wheel_encoder/odometry",  1, &EKFFusion::odomCallback,   this);
    sub_imu_    = nh_.subscribe("/automobile/IMU",                     1, &EKFFusion::imuCallback,    this);
    sub_vision_ = nh_.subscribe("/sign_detector/sign_poses",           1, &EKFFusion::visionCallback, this);

    // Publishers
    pub_odom_fused_ = nh_.advertise<nav_msgs::Odometry>("/ekf/odom", 1);
    pub_path_       = nh_.advertise<nav_msgs::Path>    ("/ekf/path", 1);

    // Initialize path message
    path_msg_.header.frame_id = "map";
  }

  void spin() {
    ros::spin();
  }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_, sub_imu_, sub_vision_;
  ros::Publisher  pub_odom_fused_, pub_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // EKF state
  Eigen::Vector3d state_;      // [x, y, theta]
  Eigen::Matrix3d cov_;        // 3×3 covariance

  // Noise
  Eigen::Matrix2d Q_;          // process noise for [v, ω]
  double          R_imu_;      // yaw measurement variance
  Eigen::Matrix2d R_vis_;      // vision [x_cam, y_cam] variance

  // Landmarks
  std::vector<Eigen::Vector2d> landmarks_;

  // Path history
  nav_msgs::Path path_msg_;

  // Callbacks
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;
    ros::Time t = odom_msg->header.stamp;

    double dt = (last_odom_time_.isZero()) 
                  ? 0.0 
                  : (t - last_odom_time_).toSec();
    last_odom_time_ = t;

    if (dt > 0) predict(v, w, dt);
    publishFused(t);
  }

    /**
   * @brief Callback for the BNO055 IMU plugin.
   *
   * The utils/IMU message provides roll, pitch, yaw (in radians)
   * along with linear acceleration (unused here). We extract yaw
   * and call updateIMU() to perform the EKF measurement update.
   * NOTE: utils/IMU has no Header, so we stamp it with ros::Time::now().
   */
  void imuCallback(const utils::IMU::ConstPtr& imu_msg)
  {
    // 1) Read yaw (heading) from the IMU message
    double meas_yaw = imu_msg->yaw;

    // (Optional) you can also access roll/pitch/accel if you extend your filter:
    // double meas_roll  = imu_msg->roll;
    // double meas_pitch = imu_msg->pitch;
    // double ax = imu_msg->accelx;
    // double ay = imu_msg->accely;
    // double az = imu_msg->accelz;

    // 2) Perform the EKF yaw update using this measurement
    //    Pass the timestamp so publishing remains synchronized.
    updateIMU(meas_yaw, ros::Time::now());
  }

  void visionCallback(const geometry_msgs::PoseArray::ConstPtr& p_arr) {
    // Transform each Pose to chassis frame (tf listener needed)
    std::vector<Eigen::Vector2d> meas;
    geometry_msgs::PoseStamped in_ps, out_ps;
    in_ps.header   = p_arr->header;
    in_ps.header.frame_id = "camera::link_camera";

    for (const auto& pose : p_arr->poses) {
      // Build a stamped Pose for tf
      in_ps.pose = pose;
      // Transform from camera frame into chassis::link
      tf_buffer_.transform(in_ps, out_ps, "chassis::link");
      // Now out_ps.pose.position.x/y are in chassis frame
      meas.emplace_back(out_ps.pose.position.x, out_ps.pose.position.y);

    }
    updateVision(meas, p_arr->header.stamp);
  }

  // EKF steps
  void predict(double v, double w, double dt) {
    // State prediction
    double theta = state_(2);
    state_(0) += v * cos(theta) * dt;
    state_(1) += v * sin(theta) * dt;
    state_(2) += w * dt;

    // Jacobians
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -v * sin(theta) * dt;
    F(1,2) =  v * cos(theta) * dt;

    Eigen::Matrix<double,3,2> B;
    B << cos(theta)*dt, 0,
         sin(theta)*dt, 0,
         0,             dt;

    // Covariance prediction
    cov_ = F * cov_ * F.transpose() + B * Q_ * B.transpose();
  }

  void updateIMU(double meas_yaw, const ros::Time& t) {
    // Measurement model: z = theta + noise
    Eigen::Vector3d H(0,0,1);
    double z = meas_yaw;
    double y = z - state_(2);
    wrapAngle(y);

    double S = H.transpose() * cov_ * H + R_imu_;
    Eigen::Vector3d K = cov_ * H / S;

    // State update
    state_ += K * y;
    state_(2) = normalizeAngle(state_(2));

    // Covariance update
    cov_ = (Eigen::Matrix3d::Identity() - K * H.transpose()) * cov_;
    
    publishFused(t);
  }

  void updateVision(const std::vector<Eigen::Vector2d>& meas, const ros::Time& t) {
    // For each landmark-measurement pair (assume same order)
    for (size_t i = 0; i < meas.size() && i < landmarks_.size(); ++i) {
      Eigen::Vector2d m = meas[i];
      Eigen::Vector2d L = landmarks_[i];

      // Predict in chassis frame: p_hat = R^T (L - [x,y])
      double theta = state_(2);
      Eigen::Rotation2Dd R(-theta);
      Eigen::Vector2d p_hat = R * (L - state_.head<2>());

      // Residual
      Eigen::Vector2d y = m - p_hat;

      // Jacobian H (2×3)
      Eigen::Matrix<double,2,3> H;
      H << -cos(theta), -sin(theta),  (L.x()*sin(theta) - L.y()*cos(theta) + state_(0)*sin(theta) - state_(1)*cos(theta)),
            sin(theta), -cos(theta), -(L.x()*cos(theta) + L.y()*sin(theta) - state_(0)*cos(theta) - state_(1)*sin(theta));

      // Innovation covariance
      Eigen::Matrix2d S = H * cov_ * H.transpose() + R_vis_;

      // Kalman gain (3×2)
      Eigen::Matrix<double,3,2> K = cov_ * H.transpose() * S.inverse();

      // Update
      state_ += K * y;
      state_(2) = normalizeAngle(state_(2));
      cov_ = (Eigen::Matrix3d::Identity() - K * H) * cov_;
    }

    publishFused(t);
  }

  // Broadcasting & publishing
  void publishFused(const ros::Time& t) {
    // 1) TF
    geometry_msgs::TransformStamped tf;
    tf.header.stamp    = t;
    tf.header.frame_id = "map";
    tf.child_frame_id  = "chassis::link";
    tf.transform.translation.x = state_(0);
    tf.transform.translation.y = state_(1);
    tf.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0,0,state_(2));
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(tf);

    // 2) Odometry message
    nav_msgs::Odometry odo;
    odo.header.stamp    = t;
    odo.header.frame_id = "map";
    odo.child_frame_id  = "chassis::link";
    odo.pose.pose.position.x = state_(0);
    odo.pose.pose.position.y = state_(1);
    odo.pose.pose.position.z = 0;
    odo.pose.pose.orientation.x = q.x();
    odo.pose.pose.orientation.y = q.y();
    odo.pose.pose.orientation.z = q.z();
    odo.pose.pose.orientation.w = q.w();
    // optionally fill covariance
    pub_odom_fused_.publish(odo);

    // 3) Path
    geometry_msgs::PoseStamped ps;
    ps.header = odo.header;
    ps.pose   = odo.pose.pose;
    path_msg_.poses.push_back(ps);
    pub_path_.publish(path_msg_);
  }

  // Utilities
  double normalizeAngle(double a) {
    while (a > M_PI)  a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }
  void wrapAngle(double& a) { a = normalizeAngle(a); }

  void loadLandmarks() {
    // Crosswalk signs
    landmarks_.emplace_back(7.312,  -3.240);   // CWALK_K
    landmarks_.emplace_back(6.627,  -4.181);   // CWALK_L
    landmarks_.emplace_back(6.777,  -1.456);   // CWALK_M
    landmarks_.emplace_back(6.130,  -2.342);   // CWALK_N
    landmarks_.emplace_back(1.109, -12.486);   // CWALK_O
    landmarks_.emplace_back(0.195, -11.869);   // CWALK_P

    // Enter-highway signs
    landmarks_.emplace_back(5.871, -13.970);   // EHIGH_T
    landmarks_.emplace_back(9.412,  -4.850);   // EHIGH_Y

    // Leave-highway signs
    landmarks_.emplace_back(10.686, -6.276);   // LHIGH_U
    landmarks_.emplace_back(6.520, -12.700);   // LHIGH_Z
    landmarks_.emplace_back(8.458, -14.359);   // LHIGH_H

    // Oneway signs
    landmarks_.emplace_back(3.341,  -9.821);   // ONEWAY_J
    landmarks_.emplace_back(2.406,  -9.821);   // ONEWAY_B
    landmarks_.emplace_back(9.196, -14.375);   // ONEWAY_E

    // Parking signs
    landmarks_.emplace_back(4.047,  -2.358);   // PRK_P1
    landmarks_.emplace_back(2.857,  -2.358);   // PRK_P2
    landmarks_.emplace_back(2.779,  -1.452);   // PRK_P3
    landmarks_.emplace_back(4.459,  -1.452);   // PRK_P4

    // Priority signs
    landmarks_.emplace_back(3.675, -13.050);   // PRIOR_D
    landmarks_.emplace_back(0.204,  -6.031);   // PRIOR_F
    landmarks_.emplace_back(5.544, -11.381);   // PRIOR_H
    landmarks_.emplace_back(4.589,  -5.997);   // PRIOR_I

    // Prohibited signs
    landmarks_.emplace_back(3.323,  -7.581);   // DENY_V
    landmarks_.emplace_back(2.387,  -7.581);   // DENY_X
    landmarks_.emplace_back(11.578, -4.504);   // DENY_G

    // Roundabout signs
    landmarks_.emplace_back(8.766,  -4.179);   // GIR_Q
    landmarks_.emplace_back(10.320, -4.869);   // GIR_R
    landmarks_.emplace_back(11.015, -3.297);   // GIR_S

    ROS_INFO("Loaded %lu landmarks", landmarks_.size());
}


  ros::Time last_odom_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_fusion_node");
  ros::NodeHandle nh("~");
  EKFFusion ekf(nh);
  ekf.spin();
  return 0;
}
