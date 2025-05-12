#include "wheel_encoder_plugin.hpp"

namespace gazebo
{

void GazeboRosWheelEncoder::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    model_ = model;
    // no sdf_ member, just use the local 'sdf' from now on

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the world file.");
        return;
    }

    // Read parameters from SDF (use the local variable 'sdf')
    if (sdf->HasElement("leftjoint"))
        joint_left_name_ = sdf->Get<std::string>("leftjoint");
    if (sdf->HasElement("rightjoint"))
        joint_right_name_ = sdf->Get<std::string>("rightjoint");

    joint_left_ = model_->GetJoint(joint_left_name_);
    joint_right_ = model_->GetJoint(joint_right_name_);
    if (!joint_left_ || !joint_right_)
    {
        ROS_ERROR_STREAM("WheelEncoderPlugin: Could not find joints '"
                         << joint_left_name_ << "' or '"
                         << joint_right_name_ << "'");
        return;
    }

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

    // Initialize timing
    last_update_time_ = model_->GetWorld()->SimTime();

    // ROS publisher
    ros_node_.reset(new ros::NodeHandle(robot_namespace_));
    odom_pub_ = ros_node_->advertise<nav_msgs::Odometry>(topic_name_, 10);

    // Connect to the world update event
    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosWheelEncoder::OnUpdate, this));
}

void GazeboRosWheelEncoder::OnUpdate()
{
    // Use the model's sim time for dt
    common::Time current_time = model_->GetWorld()->SimTime();
    double dt = (current_time - last_update_time_).Double();

    // Only publish at the configured rate
    if (dt < (1.0 / publish_rate_))
        return;

    last_update_time_ = current_time;

    // Read wheel joint velocities (rad/s)
    double vel_left  = joint_left_->GetVelocity(0);
    double vel_right = joint_right_->GetVelocity(0);

    // Convert to linear velocities (m/s)
    double v_left  = vel_left  * wheel_radius_;
    double v_right = vel_right * wheel_radius_;

    // Compute forward and yaw rates
    double linear_x  = 0.5 * (v_right + v_left);
    double angular_z = (v_right - v_left) / wheel_base_;

    // Fill and publish Odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp    = ros::Time::now();
    odom_msg.header.frame_id = "map";        // match your RViz fixed frame
    odom_msg.child_frame_id  = "base_link";  // match your robot's base link

    odom_msg.twist.twist.linear.x  = linear_x;
    odom_msg.twist.twist.angular.z = angular_z;

    // small covariance if your EKF uses it
    odom_msg.twist.covariance[0]  = 1e-4;
    odom_msg.twist.covariance[35] = 1e-4;

    odom_pub_.publish(odom_msg);
}

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelEncoder)

}  // namespace gazebo
