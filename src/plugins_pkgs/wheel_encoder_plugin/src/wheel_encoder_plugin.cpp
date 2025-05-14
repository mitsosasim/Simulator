#include "wheel_encoder_plugin.hpp"
#include <ros/console.h>

namespace gazebo
{

// define the destructor that you declared
GazeboRosWheelEncoder::~GazeboRosWheelEncoder() = default;

void GazeboRosWheelEncoder::Load(physics::ModelPtr model,
                                 sdf::ElementPtr sdf)
{
  ROS_INFO("WheelEncoderPlugin::Load() called");

  // store model ptr
  model_ = model;

  if (!ros::isInitialized())
  {
    ROS_FATAL("ROS node not initialized, load gazebo_ros_api_plugin first");
    return;
  }

  // read joint names
  if (sdf->HasElement("leftjoint"))
    joint_left_name_ = sdf->Get<std::string>("leftjoint");
  if (sdf->HasElement("rightjoint"))
    joint_right_name_ = sdf->Get<std::string>("rightjoint");

  joint_left_  = model_->GetJoint(joint_left_name_);
  joint_right_ = model_->GetJoint(joint_right_name_);
  if (!joint_left_ || !joint_right_)
  {
    ROS_ERROR("WheelEncoderPlugin: could not find joints [%s] or [%s]",
              joint_left_name_.c_str(), joint_right_name_.c_str());
    return;
  }

  // other params
  if (sdf->HasElement("wheel_radius"))
    wheel_radius_ = sdf->Get<double>("wheel_radius");
  if (sdf->HasElement("wheel_base"))
    wheel_base_ = sdf->Get<double>("wheel_base");
  if (sdf->HasElement("publish_rate"))
    publish_rate_ = sdf->Get<double>("publish_rate");
  if (sdf->HasElement("robotNamespace"))
    robot_namespace_ = sdf->Get<std::string>("robotNamespace");
  if (sdf->HasElement("topic_name"))
    topic_name_ = sdf->Get<std::string>("topic_name");

  // init timing
  last_update_time_ = model_->GetWorld()->SimTime();

  // ROS publisher
  ros_node_.reset(new ros::NodeHandle(robot_namespace_));
  odom_pub_ = ros_node_->advertise<nav_msgs::Odometry>(topic_name_, 10);

  // hook into update event
  update_conn_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosWheelEncoder::OnUpdate, this));
}

void GazeboRosWheelEncoder::OnUpdate()
{
  // compute dt
  auto current_time = model_->GetWorld()->SimTime();
  double dt = (current_time - last_update_time_).Double();
  if (dt < (1.0 / publish_rate_))
    return;
  last_update_time_ = current_time;

  // read wheel velocities
  double vel_l = joint_left_->GetVelocity(0);   // rad/s
  double vel_r = joint_right_->GetVelocity(0);  // rad/s

  // to linear
  double v_l = vel_l * wheel_radius_;
  double v_r = vel_r * wheel_radius_;

  // compute forward & yaw
  double lin_x = 0.5 * (v_r + v_l);
  double ang_z = (v_r - v_l) / wheel_base_;

  // publish
  nav_msgs::Odometry odom;
  odom.header.stamp    = ros::Time::now();
  odom.header.frame_id = "map";
  odom.child_frame_id  = "base_link";

  if (fabs(lin_x)  < 1e-3  &&  fabs(ang_z) < 1e-3)
  {
    // no movement dead-zone
    lin_x = 0.0;
    ang_z = 0.0;
  }
 
  
  odom.twist.twist.linear.x  = lin_x;
  odom.twist.twist.angular.z = ang_z;

  odom.twist.covariance[0]   = 1e-4;
  odom.twist.covariance[35]  = 1e-4;

  odom_pub_.publish(odom);
}

// Finally, register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelEncoder)

}  // namespace gazebo
