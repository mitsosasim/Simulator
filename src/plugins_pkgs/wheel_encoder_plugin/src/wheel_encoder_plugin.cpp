// -------------------------------------------------------
// wheel_encoder_plugin.cpp
//   (Ackermann steering odometry plugin implementation)
// -------------------------------------------------------

#include "wheel_encoder_plugin.hpp"
#include <ros/console.h>

// Gazebo core / SDF
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{

// Destructor
GazeboRosWheelEncoder::~GazeboRosWheelEncoder() = default;

void GazeboRosWheelEncoder::Load(physics::ModelPtr model,
                                 sdf::ElementPtr sdf)
{
  ROS_INFO("AckermannWheelEncoderPlugin::Load() called");

  // 1) Keep a pointer to the parent Model
  model_ = model;

  // 2) Ensure ROS has been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL("ROS node not initialized; load gazebo_ros_api_plugin first");
    return;
  }

  // 3) Parse required SDF parameters:

  //   a) Rear‐wheel joint names
  if (sdf->HasElement("leftjoint"))
    rear_left_wheel_name_ = sdf->Get<std::string>("leftjoint");
  else
  {
    ROS_ERROR("AckermannWheelEncoderPlugin: <leftjoint> missing");
    return;
  }
  if (sdf->HasElement("rightjoint"))
    rear_right_wheel_name_ = sdf->Get<std::string>("rightjoint");
  else
  {
    ROS_ERROR("AckermannWheelEncoderPlugin: <rightjoint> missing");
    return;
  }

  //   b) Front‐steer joint names
  if (sdf->HasElement("steer_left_joint"))
    steer_left_name_ = sdf->Get<std::string>("steer_left_joint");
  else
  {
    ROS_ERROR("AckermannWheelEncoderPlugin: <steer_left_joint> missing");
    return;
  }
  if (sdf->HasElement("steer_right_joint"))
    steer_right_name_ = sdf->Get<std::string>("steer_right_joint");
  else
  {
    ROS_ERROR("!!!!!!!!!!!!!!!AckermannWheelEncoderPlugin: <steer_right_joint> missing");
    return;
  }

  //   c) Wheel geometry
  if (sdf->HasElement("wheel_radius"))
    wheel_radius_ = sdf->Get<double>("wheel_radius");
  else
  {
    ROS_ERROR("AckermannWheelEncoderPlugin: <wheel_radius> missing");
    return;
  }
  if (sdf->HasElement("wheel_base"))
    wheel_base_ = sdf->Get<double>("wheel_base");
  else
  {
    ROS_ERROR("AckermannWheelEncoderPlugin: <wheel_base> missing");
    return;
  }

  //   d) Rate, namespace, topic
  if (sdf->HasElement("publish_rate"))
    publish_rate_ = sdf->Get<double>("publish_rate");
  else
    publish_rate_ = 50.0;  // default 50 Hz

  if (sdf->HasElement("robotNamespace"))
    robot_namespace_ = sdf->Get<std::string>("robotNamespace");
  else
    robot_namespace_ = "";

  if (sdf->HasElement("topic_name"))
    topic_name_ = sdf->Get<std::string>("topic_name");
  else
    topic_name_ = "/automobile/wheel_encoder/odometry";

  // 4) Retrieve the actual Joint pointers from the model
  rear_left_wheel_   = model_->GetJoint(rear_left_wheel_name_);
  rear_right_wheel_  = model_->GetJoint(rear_right_wheel_name_);
  steer_left_wheel_  = model_->GetJoint(steer_left_name_);
  steer_right_wheel_ = model_->GetJoint(steer_right_name_);

  if (!rear_left_wheel_ || !rear_right_wheel_
      || !steer_left_wheel_ || !steer_right_wheel_)
  {
    ROS_ERROR("AckermannWheelEncoderPlugin: joint not found: [%s] [%s] [%s] [%s]",
      rear_left_wheel_name_.c_str(),
      rear_right_wheel_name_.c_str(),
      steer_left_name_.c_str(),
      steer_right_name_.c_str());
    return;
  }

  // 5) Initialize timing
  last_update_time_ = model_->GetWorld()->SimTime();

  // 6) Set up ROS publisher
  ros_node_.reset(new ros::NodeHandle(robot_namespace_));
  odom_pub_ = ros_node_->advertise<nav_msgs::Odometry>(topic_name_, 10);

  // 7) Hook into the Gazebo world update event
  update_conn_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosWheelEncoder::OnUpdate, this)
  );

  ROS_INFO(
    "AckermannWheelEncoderPlugin loaded:\n"
    "  Rear wheels: [%s], [%s]\n"
    "  Steer joints: [%s], [%s]\n"
    "  wheel_radius=%.4f, wheel_base=%.4f\n"
    "  publish_rate=%.1f Hz → topic [%s]",
    rear_left_wheel_name_.c_str(),
    rear_right_wheel_name_.c_str(),
    steer_left_name_.c_str(),
    steer_right_name_.c_str(),
    wheel_radius_, wheel_base_,
    publish_rate_, topic_name_.c_str()
  );
}

void GazeboRosWheelEncoder::OnUpdate()
{
  // 1) Throttle by publish_rate_
  common::Time current_time = model_->GetWorld()->SimTime();
  double dt = (current_time - last_update_time_).Double();
  if (dt < (1.0 / publish_rate_))
    return;
  last_update_time_ = current_time;

  // 2) Read rear‐wheel velocities (rad/s)
  double vel_l = rear_left_wheel_->GetVelocity(0);
  double vel_r = rear_right_wheel_->GetVelocity(0);

  // 3) Convert to forward speed (m/s)
  double v_l  = vel_l * wheel_radius_;
  double v_r  = vel_r * wheel_radius_;
  double lin_x = 0.5 * (v_l + v_r);

  // 4) Read front‐steer angles (rad)
  double delta_l = steer_left_wheel_->Position(0);
  double delta_r = steer_right_wheel_->Position(0);

  // 5) Compute effective steering angle δ
  double steer_angle = 0.5 * (delta_l + delta_r);

  // 6) Compute yaw rate ω = v * tan(δ) / L
  double ang_z = 0.0;
  if (fabs(steer_angle) > 1e-6)
    ang_z = lin_x * std::tan(steer_angle) / wheel_base_;
  else
    ang_z = 0.0;

  // 7) Dead‐zone cleanup
  if (fabs(lin_x) < 1e-3 && fabs(ang_z) < 1e-3)
  {
    lin_x = 0.0;
    ang_z = 0.0;
  }

  // 8) Publish nav_msgs::Odometry
  nav_msgs::Odometry odom;
  odom.header.stamp    = ros::Time::now();
  odom.header.frame_id = "map";
  odom.child_frame_id  = "chassis::link";

  odom.twist.twist.linear.x  = lin_x;
  odom.twist.twist.angular.z = ang_z;

  // Covariance entries (tune if necessary)
  odom.twist.covariance[0]  = 1e-4;  // variance for v
  odom.twist.covariance[35] = 1e-4;  // variance for ω

  odom_pub_.publish(odom);
}

// Register this plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelEncoder)

}  // namespace gazebo
