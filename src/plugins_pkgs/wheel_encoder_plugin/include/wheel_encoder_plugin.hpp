#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
/// \brief A plugin to simulate wheel encoders on an Ackermann drive vehicle,
///        publishing odometry twist based on rear wheel velocities.
class GazeboRosWheelEncoder : public ModelPlugin
{
public:
GazeboRosWheelEncoder() = default;
virtual ~GazeboRosWheelEncoder();


/// \brief Load the plugin, read parameters from SDF, and initialize ROS publisher.
/// \param[in] _model Pointer to the parent model.
/// \param[in] _sdf   SDF element containing plugin parameters.
void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;


private:
/// \brief Called every simulation iteration to compute and publish encoder data.
void OnUpdate();

/// Pointer to the model
physics::ModelPtr model_;
/// Pointer to the update event connection
event::ConnectionPtr update_conn_;

/// ROS node handle
std::unique_ptr<ros::NodeHandle> ros_node_;
/// ROS odometry publisher
ros::Publisher odom_pub_;

/// Names of the two drive wheel joints
/// Names of the two drive wheel joints
// std::string joint_leftrear_rim;
// std::string joint_rightrear_rim;


/// SDF-loaded names of the two drive wheel joints
std::string joint_left_name_;
std::string joint_right_name_;


/// Pointers to the drive wheel joints
physics::JointPtr joint_left_;
physics::JointPtr joint_right_;

/// Wheel and vehicle parameters
double wheel_radius_ = 0.0325;   ///< in meters
double wheel_base_ = 0.26;     ///< distance between left and right wheels (m)
double publish_rate_ = 100;   ///< in Hz
std::string topic_name_ = "/automobile/wheel_encoder/odometry"; ///< topic name
std::string robot_namespace_ ="/";

/// State storage for previous angles and time
common::Time last_update_time_;
double prev_angle_left_  = 0.0;
double prev_angle_right_ = 0.0;

/// Utility: convert Gazebo time to ROS time
ros::Time GazeboTimeToRos(const common::Time &t) const;


};
} // namespace gazebo
