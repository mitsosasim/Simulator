#include "traffic_light_plugin.hpp"

#define DEBUG false

namespace gazebo
{
    namespace trafficLight
    {   
        TrafficLight::TrafficLight():ModelPlugin() {}

        void TrafficLight::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {        	
			// Save a pointer to the model for later use
            this->m_ros_node = gazebo_ros::Node::Get(sdf_ptr);
			this->m_model = model_ptr;
			this->name = this->m_model->GetName();
			
        	// Create transport node
			this->m_node = transport::NodePtr(new transport::Node());
			this->m_node->Init();
        	
            // TODO: I don't see "~/light/modify" anywhere else in the codebase, where is it used?
        	this->m_pubLight = this->m_node->Advertise<gazebo::msgs::Light>("~/light/modify");
        	
        	// Save pointers to each link of the traffic light (aka lens)
        	this->m_green_lens_link		= this->m_model->GetLink("green_lens");
			this->m_yellow_lens_link	= this->m_model->GetLink("yellow_lens");
			this->m_red_lens_link	 	= this->m_model->GetLink("red_lens");
      	
			// Create a subscriber
			this->m_ros_subscriber = this->m_ros_node->create_subscription<std_msgs::msg::Byte>("/automobile/trafficlight/" + this->name, 1, std::bind(&trafficLight::TrafficLight::OnRosMsg, this, std::placeholders::_1));

			if(DEBUG)
            {
                auto logger = this->m_ros_node->get_logger();
                std::cerr << "\n\n";
                RCLCPP_INFO_STREAM(logger, "====================================================================");
                RCLCPP_INFO_STREAM(logger, "[traffic_light_plugin] attached to: " << this->name);
                RCLCPP_INFO_STREAM(logger, "[traffic_light_plugin] listen to: /traffic_light_topic ");
                RCLCPP_INFO_STREAM(logger, "====================================================================");
            }
        }

		// defining the traffic_light states
        void TrafficLight::redState(const bool state)
        {
        	this->m_msg.set_name(this->m_red_lens_link->GetScopedName() + "::" + static_cast<std::string>("red_lens_light"));
		 	this->m_msg.set_range(state? 5 : 0);
			this->m_pubLight->Publish(m_msg);
        }
        
        void TrafficLight::yellowState(const bool state)
        {
        	this->m_msg.set_name(this->m_yellow_lens_link->GetScopedName() + "::" + static_cast<std::string>("yellow_lens_light"));
		 	this->m_msg.set_range(state? 5 : 0);
			this->m_pubLight->Publish(m_msg);
        }
        void TrafficLight::greenState(const bool state)
        
        {
        	this->m_msg.set_name(this->m_green_lens_link->GetScopedName() + "::" + static_cast<std::string>("green_lens_light"));
		 	this->m_msg.set_range(state? 5 : 0);
			this->m_pubLight->Publish(m_msg);
        }
        
        // Handle an incoming message from ROS
		// _msg A float value that is used to set the velocity
		void TrafficLight::OnRosMsg(std_msgs::msg::Byte _msg)
		{
			switch(_msg.data)
			{
				case TrafficLightColor::RED:
					redState(1); yellowState(0); greenState(0);
					break;

				case TrafficLightColor::YELLOW:
					redState(0); yellowState(1); greenState(0);
					break;

				case TrafficLightColor::GREEN:
					redState(0); yellowState(0); greenState(1);
					break;
			}
		};
    }; //namespace trafficLight
    GZ_REGISTER_MODEL_PLUGIN(trafficLight::TrafficLight)
}; // namespace gazebo
