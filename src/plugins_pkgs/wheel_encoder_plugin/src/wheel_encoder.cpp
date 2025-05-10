#include "wheel_encoder.hpp"

namespace gazebo
{

void GazeboRosWheelEncoder::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    model_ = model;
    sdf_ = sdf;

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the world file.");
        return;
    }

    // Read parameters from SDF
    if (sdf_->HasElement("leftjoint"))
        joint_left_name_ = sdf_->Get<std::string>("leftjoint");
    if (sdf_->HasElement("rightjoint"))
        joint_right_name_ = sdf_->Get<std::string>("rightjoint");

    joint_left_ = model_->GetJoint(joint_left_name_);
    joint_right_ = model_->GetJoint(joint_right_name_);

    if (!joint_left_ || !joint_right_)
    {
        ROS_ERROR_STREAM("WheelEncoderPlugin: Could not find specified joints: "
                         << joint_left_name_ << ", " << joint_right_name_);
        return;
    }

    if (sdf_->HasElement("wheel_radius"))
        wheel_radius_ = sdf_->Get<double>("wheel_radius");

    if (sdf_->HasElement("wheel_base"))
        wheel_base_ = sdf_->Get<double>("wheel_base");

    if (sdf_->HasElement("publish_rate"))
        publish_rate_ = sdf_->Get<double>("publish_rate");

    if (sdf_->HasElement("robotNamespace"))
        robot_namespace_ = sdf_->Get<std::string>("robotNamespace");

    if (sdf_->HasElement("topic_name"))
        topic_name_ = sdf_->Get<std::string>("topic_name");

    last_pub_time_ = model_->GetWorld()->SimTime();

    ros_node_.reset(new ros::NodeHandle(robot_namespace_));
    odom_pub_ = ros_node_->advertise<nav_msgs::Odometry>(topic_name_, 10);

    // Connect to the world update event
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosWheelEncoder::OnUpdate, this));
}

void GazeboRosWheelEncoder::OnUpdate()
{
    common::Time current_time = model_->GetWorld()->SimTime();
    double dt = (current_time - last_pub_time_).Double();

    if (dt < (1.0 / publish_rate_))
        return;

    last_pub_time_ = current_time;

    double vel_left = joint_left_->GetVelocity(0);  // rad/s
    double vel_right = joint_right_->GetVelocity(0); // rad/s

    double v_left = vel_left * wheel_radius_;
    double v_right = vel_right * wheel_radius_;

    double linear_x = (v_right + v_left) / 2.0;
    double angular_z = (v_right - v_left) / wheel_base_;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.angular.z = angular_z;

    // Optionally, add covariance if your EKF expects it
    odom_msg.twist.covariance[0] = 0.0001;
    odom_msg.twist.covariance[35] = 0.0001;

    odom_pub_.publish(odom_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelEncoder)

}  // namespace gazebo
