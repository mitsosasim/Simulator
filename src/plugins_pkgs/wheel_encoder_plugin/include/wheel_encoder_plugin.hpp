#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <functional>
#include <memory>

namespace gazebo
{

/// \brief A plugin to simulate wheel encoders on an Ackermann drive vehicle,
///        publishing odometry twist based on rear wheel velocities.
class GazeboRosWheelEncoder : public ModelPlugin
{
public:
  GazeboRosWheelEncoder() = default;
  virtual ~GazeboRosWheelEncoder();  // declare

  /// \brief Load the plugin, read parameters from SDF, and initialize ROS.
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Called every simulation iteration to compute and publish encoder data.
  void OnUpdate();

  /// Utility: convert Gazebo time to ROS time (optional; we use ros::Time::now())
  ros::Time GazeboTimeToRos(const common::Time &t) const
  {
    return ros::Time(t.sec, t.nsec);
  }

  // model & update hook
  physics::ModelPtr      model_;
  event::ConnectionPtr   update_conn_;

  // ROS
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher                   odom_pub_;

  // joint names from SDF
  std::string joint_left_name_;
  std::string joint_right_name_;

  // actual JointPtr
  physics::JointPtr joint_left_;
  physics::JointPtr joint_right_;

  // parameters
  double          wheel_radius_     = 0.0325;
  double          wheel_base_       = 0.26;
  double          publish_rate_     = 100.0;
  std::string     topic_name_       = "/automobile/wheel_encoder/odometry";
  std::string     robot_namespace_  = "/";

  // timing
  common::Time    last_update_time_;
};

}  // namespace gazebo
