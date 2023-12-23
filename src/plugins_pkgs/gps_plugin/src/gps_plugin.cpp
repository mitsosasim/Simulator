
#include "gps_plugin.hpp"

#define DEBUG false

namespace gazebo
{
    namespace gps
    {   
        GPS::GPS():ModelPlugin() {}
     		
        void GPS::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
            this->m_model = model_ptr;
            this->m_ros_node = gazebo_ros::Node::Get(sdf_ptr);
            this->timer = this->m_ros_node->create_wall_timer(std::chrono::milliseconds(250), std::bind(&GPS::OnUpdate, this));
  			
            std::string topic_name = "/automobile/localisation";
  	        
        	this->m_pubGPS = this->m_ros_node->create_publisher<utils::msg::Localisation>(topic_name, 2);
        	
          if(DEBUG)
          {
              auto logger = this->m_ros_node->get_logger();
              std::cerr << "\n\n";
              RCLCPP_INFO_STREAM(logger, "====================================================================");
              RCLCPP_INFO_STREAM(logger, "[gps_plugin] attached to: " << this->m_model->GetName());
              RCLCPP_INFO_STREAM(logger, "[gps_plugin] publish to: "  << topic_name);
              RCLCPP_INFO_STREAM(logger, "[gps_plugin] Usefull data: linear x, linear y, angular z");
              RCLCPP_INFO_STREAM(logger, "====================================================================\n\n");
          }
        }

        // Publish the updated values
        void GPS::OnUpdate()
        {
            this->m_gps_pose.timestamp  = this->m_model->GetWorld()->SimTime().Float();
           	this->m_gps_pose.pos_a   = this->m_model->RelativePose().Pos().X() + (rand() / (float)RAND_MAX * 0.2) - 0.1;
           	this->m_gps_pose.pos_b   = abs(this->m_model->RelativePose().Pos().Y()) + (rand() / (float)RAND_MAX * 0.2) - 0.1;
           	this->m_gps_pose.rot_a   = this->m_model->RelativePose().Rot().Yaw();
           	this->m_gps_pose.rot_b   = this->m_model->RelativePose().Rot().Yaw();
            this->m_pubGPS->publish(this->m_gps_pose);
        };      
    }; //namespace trafficLight
    GZ_REGISTER_MODEL_PLUGIN(gps::GPS)
}; // namespace gazebo
