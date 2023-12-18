#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"
#include <utils/msg/imu.hpp>


namespace gazebo
{
    namespace bno055
    {   
        class BNO055: public ModelPlugin
    	{
        private: 
            physics::ModelPtr m_model;
            rclcpp::TimerBase::SharedPtr timer;
            rclcpp::Node::SharedPtr m_ros_node;
            rclcpp::Publisher<utils::msg::IMU>::SharedPtr m_pubBNO;
            utils::msg::IMU m_bno055_pose;

        // Default constructor
        public: BNO055();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();        
        };
    };    
};
