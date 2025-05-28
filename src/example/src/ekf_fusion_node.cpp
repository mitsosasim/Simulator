#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <utils/IMU.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ---------- NEW INCLUDES ----------
#include <utils/SignLabelArray.h>        // your custom msg
#include <std_msgs/String.h>               // not strictly needed here, but kept if you use strings elsewhere
// ----------------------------------


class EKFFusion
{
public:
  EKFFusion(ros::NodeHandle& nh)
    : nh_(nh),
      tf_listener_(tf_buffer_),
      state_(Eigen::Vector3d::Zero()),
      cov_(Eigen::Matrix3d::Identity() * 1e-3),
      last_odom_time_(ros::Time(0))
  {
    loadLandmarks();  // now loads class‐tagged landmarks

    // noise, subscribers, publishers (unchanged)…
    Q_ = Eigen::Matrix2d::Identity() * 1e-4;      
    R_imu_ = 1e-2;                                
    R_vis_ = Eigen::Matrix2d::Identity() * 1e-1;  

    sub_odom_   = nh_.subscribe("/automobile/wheel_encoder/odometry", 1, &EKFFusion::odomCallback, this);
    sub_imu_    = nh_.subscribe("/automobile/IMU",                    1, &EKFFusion::imuCallback,  this);

    // subscribe to camera‐based poses AND to your new label array:
    sub_vision_ = nh_.subscribe("/sign_detector/sign_poses",  1, &EKFFusion::visionCallback,    this);
    sub_labels_ = nh_.subscribe("/sign_detector/sign_labels",1, &EKFFusion::visionLabelCb,    this);

    sub_gt_path_ = nh_.subscribe("/ground_truth_path", 1, &EKFFusion::groundTruthCallback, this);

    pub_odom_fused_ = nh_.advertise<nav_msgs::Odometry>("/ekf/odom", 1);
    pub_path_       = nh_.advertise<nav_msgs::Path>    ("/ekf/path", 1);
    path_msg_.header.frame_id = "map";
  }

  void spin() { ros::spin(); }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_, sub_imu_, sub_vision_, sub_labels_, sub_gt_path_;
  ros::Publisher  pub_odom_fused_, pub_path_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // buffers for paired vision data
  std::vector<geometry_msgs::Pose> latest_poses_;
  std::vector<std::string>         latest_labels_;

  bool initialized_ = false;
  ros::Time last_odom_time_;

  // EKF state & noise
  Eigen::Vector3d state_;
  Eigen::Matrix3d cov_;
  Eigen::Matrix2d Q_;
  double          R_imu_;
  Eigen::Matrix2d R_vis_;

  // A landmark entry now carries its class tag
  struct Landmark { std::string cls; Eigen::Vector2d pos; };
  std::vector<Landmark> landmarks_;

  nav_msgs::Path path_msg_;

  // --- callbacks & helpers ---

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    if (!initialized_) return;
    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;
    ros::Time t = odom_msg->header.stamp;
    double dt = last_odom_time_.isZero() ? 0.0 : (t - last_odom_time_).toSec();
    last_odom_time_ = t;
    if (dt > 0) predict(v,w,dt);
    publishFused(t);
  }

  void imuCallback(const utils::IMU::ConstPtr& imu_msg) {
    if (!initialized_) return;
    updateIMU(imu_msg->yaw, ros::Time::now());
  }

  void visionCallback(const geometry_msgs::PoseArray::ConstPtr& p_arr) {
    if (!initialized_) return;
    // buffer and transform into chassis frame
    latest_poses_.clear();
    geometry_msgs::PoseStamped in, out;
    in.header = p_arr->header;
    in.header.frame_id = "camera::link_camera";
    for (auto& ps : p_arr->poses) {
      in.pose = ps;
      tf_buffer_.transform(in, out, "chassis::link");
      latest_poses_.push_back(out.pose);
    }
    tryFuseVision(p_arr->poses.size());
  }

  void visionLabelCb(const example::SignLabelArray::ConstPtr& msg) {
    latest_labels_ = msg->labels;
    tryFuseVision(msg->labels.size());
  }

  void tryFuseVision(size_t expected) {
    if (latest_poses_.size()!=expected || latest_labels_.size()!=expected) return;
    // build 2D meas
    std::vector<Eigen::Vector2d> meas2d;
    for (auto& ps: latest_poses_)
      meas2d.emplace_back(ps.position.x, ps.position.y);
    // fused update with paired labels
    updateVision(meas2d, latest_labels_, ros::Time::now());
    latest_poses_.clear();
    latest_labels_.clear();
  }

  void groundTruthCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    if (initialized_ || path_msg->poses.empty()) return;
    auto& ps = path_msg->poses.front();
    state_(0)=ps.pose.position.x; state_(1)=ps.pose.position.y;
    tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y,
                      ps.pose.orientation.z, ps.pose.orientation.w);
    double r,p,y; tf2::Matrix3x3(q).getRPY(r,p,y);
    state_(2)=y; last_odom_time_=ps.header.stamp; initialized_=true;
    ROS_INFO("EKF init ⟶ x=%.3f y=%.3f yaw=%.3f",state_(0),state_(1),state_(2));
    sub_gt_path_.shutdown();
  }

  void predict(double v, double w, double dt) {
    double θ=state_(2);
    state_(0)+=v*cos(θ)*dt; state_(1)+=v*sin(θ)*dt; state_(2)+=w*dt;
    Eigen::Matrix3d F=Eigen::Matrix3d::Identity();
    F(0,2) = -v*sin(θ)*dt; F(1,2)=v*cos(θ)*dt;
    Eigen::Matrix<double,3,2> B; B<<cos(θ)*dt,0, sin(θ)*dt,0, 0,dt;
    cov_ = F*cov_*F.transpose() + B*Q_*B.transpose();
  }

  void updateIMU(double meas_yaw, const ros::Time& t) {
    Eigen::Vector3d H(0,0,1);
    double z=meas_yaw, y=z-state_(2); wrapAngle(y);
    double S=H.transpose()*cov_*H + R_imu_;
    Eigen::Vector3d K=cov_*H/S;
    state_ += K*y; state_(2)=normalizeAngle(state_(2));
    cov_ = (Eigen::Matrix3d::Identity()-K*H.transpose())*cov_;
    publishFused(t);
  }

  // ----------------- NEW overloaded updateVision -----------------
  void updateVision(const std::vector<Eigen::Vector2d>& meas,
                    const std::vector<std::string>& labels,
                    const ros::Time& t)
  {
    for (size_t i=0; i<meas.size(); ++i) {
      const auto& m = meas[i];
      const auto& cls = labels[i];
      // find nearest landmark of this class
      double best_d = 1e9; Eigen::Vector2d L;
      for (auto& lm: landmarks_) {
        if (lm.cls!=cls) continue;
        double d=(lm.pos-m).norm();
        if (d<best_d) { best_d=d; L=lm.pos; }
      }
      if (best_d>5.0) continue;  // skip if too far (tunable!)

      double θ=state_(2);
      Eigen::Rotation2Dd R(-θ);
      Eigen::Vector2d p_hat = R*(L - state_.head<2>());
      Eigen::Vector2d y = m - p_hat;

      Eigen::Matrix<double,2,3> H;
      H << -cos(θ), -sin(θ),  (L.x()*sin(θ)-L.y()*cos(θ)+state_(0)*sin(θ)-state_(1)*cos(θ)),
             sin(θ), -cos(θ), -(L.x()*cos(θ)+L.y()*sin(θ)-state_(0)*cos(θ)-state_(1)*sin(θ));

      Eigen::Matrix2d S = H*cov_*H.transpose() + R_vis_;
      Eigen::Matrix<double,3,2> K = cov_*H.transpose()*S.inverse();

      state_ += K*y; state_(2)=normalizeAngle(state_(2));
      cov_ = (Eigen::Matrix3d::Identity() - K*H)*cov_;
    }
    publishFused(t);
  }
  // ---------------------------------------------------------------

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
  double normalizeAngle(double a) {
    while(a>M_PI) a-=2*M_PI;
    while(a<-M_PI) a+=2*M_PI;
    return a;
  }
  void wrapAngle(double& a) { a=normalizeAngle(a); }

  void loadLandmarks() {
    // Crosswalk signs
    landmarks_.push_back({"crosswalk", Eigen::Vector2d(7.312, -3.240)});   // CWALK_K
    landmarks_.push_back({"crosswalk", Eigen::Vector2d(6.627, -4.181)});   // CWALK_L
    landmarks_.push_back({"crosswalk", Eigen::Vector2d(6.777, -1.456)});   // CWALK_M
    landmarks_.push_back({"crosswalk", Eigen::Vector2d(6.130, -2.342)});   // CWALK_N
    landmarks_.push_back({"crosswalk", Eigen::Vector2d(1.109, -12.486)});   // CWALK_O
    landmarks_.push_back({"crosswalk", Eigen::Vector2d(0.195, -11.869)});   // CWALK_P
    // Enter-highway signs
    landmarks_.push_back({"enterhighway", Eigen::Vector2d(5.871, -13.970)});   // EHIGH_T
    landmarks_.push_back({"enterhighway", Eigen::Vector2d(9.412, -4.850)});   // EHIGH_Y
    // Leave-highway signs
    landmarks_.push_back({"leavehighway", Eigen::Vector2d(10.686, -6.276)});   // LHIGH_U
    landmarks_.push_back({"leavehighway", Eigen::Vector2d(6.520, -12.700)});   // LHIGH_Z
    landmarks_.push_back({"leavehighway", Eigen::Vector2d(8.458, -14.359)});   // LHIGH_H
    // Oneway signs
    landmarks_.push_back({"oneway", Eigen::Vector2d(3.341, -9.821)});   // ONEWAY_J
    landmarks_.push_back({"oneway", Eigen::Vector2d(2.406, -9.821)});   // ONEWAY_B
    landmarks_.push_back({"oneway", Eigen::Vector2d(9.196, -14.375)});   // ONEWAY_E
    // Parking signs
    landmarks_.push_back({"parking", Eigen::Vector2d(4.047, -2.358)});   // PRK_P1
    landmarks_.push_back({"parking", Eigen::Vector2d(2.857, -2.358)});   // PRK_P2
    landmarks_.push_back({"parking", Eigen::Vector2d(2.779, -1.452)});   // PRK_P3
    landmarks_.push_back({"parking", Eigen::Vector2d(4.459, -1.452)});   // PRK_P4
    // Priority signs
    landmarks_.push_back({"priority", Eigen::Vector2d(3.675, -13.050)});   // PRIOR_D
    landmarks_.push_back({"priority", Eigen::Vector2d(0.204, -6.031)});   // PRIOR_F
    landmarks_.push_back({"priority", Eigen::Vector2d(5.544, -11.381)});   // PRIOR_H
    landmarks_.push_back({"priority", Eigen::Vector2d(4.589, -5.997)});   // PRIOR_I
    // Prohibited signs
    landmarks_.push_back({"prohibited", Eigen::Vector2d(3.323, -7.581)});   // DENY_V
    landmarks_.push_back({"prohibited", Eigen::Vector2d(2.387, -7.581)});   // DENY_X
    landmarks_.push_back({"prohibited", Eigen::Vector2d(11.578, -4.504)});   // DENY_G
    // Roundabout signs
    landmarks_.push_back({"roundabout", Eigen::Vector2d(8.766, -4.179)});   // GIR_Q
    landmarks_.push_back({"roundabout", Eigen::Vector2d(10.320, -4.869)});   // GIR_R
    landmarks_.push_back({"roundabout", Eigen::Vector2d(11.015, -3.297)});   // GIR_S
 


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_fusion_node");
  ros::NodeHandle nh("~");
  EKFFusion ekf(nh);
  ekf.spin();
  return 0;
}
