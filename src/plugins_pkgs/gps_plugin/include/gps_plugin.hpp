#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"
#include <utils/msg/localisation.hpp>

namespace gazebo
{
    namespace gps
    {   
        class GPS: public ModelPlugin
    	{
        private: 
            physics::ModelPtr m_model;
            rclcpp::TimerBase::SharedPtr timer;
            rclcpp::Node::SharedPtr m_ros_node;
            rclcpp::Publisher<utils::msg::Localisation>::SharedPtr m_pubGPS;
            utils::msg::Localisation m_gps_pose;

        // Default constructor
        public: GPS();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();        
        };
    };    
};
