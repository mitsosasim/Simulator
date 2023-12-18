#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

#include "carlikerobot.hpp"
#include "rapidjson/document.h"

namespace gazebo
{
	namespace carlikerobot
    {

        class IRobotCommandSetter
        {
            public:
                virtual void setCommand()=0;
                float f_steer;
                float f_speed;
        };       
        
        typedef std::shared_ptr<IRobotCommandSetter> IRobotCommandSetterPtr;

        class CMessageHandler
        {
            public:
                CMessageHandler(rclcpp::Node::SharedPtr, IRobotCommandSetter*);
                ~CMessageHandler();
                void OnMsgCommand(const std_msgs::msg::String::SharedPtr _msg);

            private:
                void unknownMessage();
                void brakeMessage(float _msg_val);
                void spedMessage(float _msg_val);
                void sterMessage(float _msg_val);
                
                IRobotCommandSetter* _robotSetter;
                rclcpp::Node::SharedPtr _rosNode;
                rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _commandSubscriber;
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _feedbackPublisher;
		
        };
        
        
        class CCarLikeRobotRosPlugin: public ModelPlugin, public IRobotCommandSetter
        {
            
            public:
                bool LoadParameterJoints(sdf::ElementPtr _sdf);
                void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
                void setCommand();
            private:
                // Private variable
                IWheelsSpeedPtr     _rearWheelsSpeedPtr;
                IWheelsSpeedPtr     _frontWheelSpeedPtr;
                ISteerWheelsPtr     _steerWheelsAnglePtr;
                
                physics::ModelPtr   _model;                

                std::shared_ptr<CMessageHandler>      _messageHandler;
                
        };	
	}; // namespace carlikerobot
}; // namespace gazebo
