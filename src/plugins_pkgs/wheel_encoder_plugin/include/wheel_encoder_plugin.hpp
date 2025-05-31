#ifndef WHEEL_ENCODER_PLUGIN_HPP
#define WHEEL_ENCODER_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <memory>
#include <string>

namespace gazebo
{
  /// \brief  Publishes Ackermann‐corrected odometry (v, ω) as nav_msgs/Odometry.
  class GazeboRosWheelEncoder : public ModelPlugin
  {
  public:
    GazeboRosWheelEncoder() = default;
    virtual ~GazeboRosWheelEncoder();

    /// \brief  Load is called once when the plugin is inserted into the simulator.
    /// \param[in] model  Pointer to the parent model
    /// \param[in] sdf    SDF element containing plugin parameters
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

    /// \brief  Called on every simulation iteration (as long as publish_rate allows)
    void OnUpdate();

  private:
    // Pointer to the parent model
    physics::ModelPtr model_;

    // ROS node & publisher
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Publisher odom_pub_;

    // Connection to the world update event
    event::ConnectionPtr update_conn_;

    // ------------------------------
    // Parameter names read from SDF
    // ------------------------------

    /// Joint names for the two rear wheels (to compute forward speed)
    std::string rear_left_wheel_name_;
    std::string rear_right_wheel_name_;

    /// Joint names for the two front steering joints (to compute steer angle)
    std::string steer_left_name_;
    std::string steer_right_name_;

    /// Geometry: wheel radius (m) and wheel base (m)
    double wheel_radius_;
    double wheel_base_;

    /// How often (Hz) to publish odometry
    double publish_rate_;

    /// ROS namespace and topic name for odometry
    std::string robot_namespace_;
    std::string topic_name_;

    // ---------------------------------------
    // Pointers to the actual Gazebo Joints
    // ---------------------------------------
    physics::JointPtr rear_left_wheel_;
    physics::JointPtr rear_right_wheel_;
    physics::JointPtr steer_left_wheel_;
    physics::JointPtr steer_right_wheel_;

    // Last time we published (used to throttle by publish_rate_)
    gazebo::common::Time last_update_time_;
  };
}

#endif  // WHEEL_ENCODER_PLUGIN_HPP
