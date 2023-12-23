#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/byte.hpp>

#pragma once

namespace gazebo
{
    namespace trafficLight
    {   
        enum TrafficLightColor {RED, YELLOW, GREEN};

        class TrafficLight: public ModelPlugin
    	{
        private: 
            physics::ModelPtr m_model;
            rclcpp::Node::SharedPtr m_ros_node;
            rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr m_ros_subscriber;
            std_msgs::msg::Byte                      m_traffic_light_msg;
            physics::LinkPtr                    m_green_lens_link;
            physics::LinkPtr                    m_yellow_lens_link;
            physics::LinkPtr                    m_red_lens_link;
            std::string                         name;

        public: transport::PublisherPtr         m_pubLight;
        public: msgs::Light                     m_msg;
        public: transport::NodePtr              m_node;

        // Default constructor
        public: TrafficLight();
        public: void Load(physics::ModelPtr, sdf::ElementPtr);
        public: void OnRosMsg(std_msgs::msg::Byte);
        private: void redState(const bool state=0);
        private: void greenState(const bool state=0);
        private: void yellowState(const bool state=0);
        
        };
    };    
};
