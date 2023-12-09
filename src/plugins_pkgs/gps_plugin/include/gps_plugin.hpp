#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>


#include "rclcpp/rclcpp.hpp"
#include <utils/msg/localisation.hpp>

namespace gazebo
{
    namespace gps
    {   
        class GPS: public ModelPlugin
    	{
        private: 
            physics::ModelPtr                   m_model;
            rclcpp::NodePtr		  nh;
            rclcpp::Timer				  timer;

	    /** ROS INTEGRATION **/
            // A node use for ROS transport
            std::unique_ptr<rclcpp::Node>    m_ros_node;

            // A ROS publisher
            ros::Publisher                      m_pubGPS;

            // The gps message
            utils::msg::localisation            m_gps_pose;
            
        // Default constructor
        public: GPS();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnUpdate();        
        };
    };    
};