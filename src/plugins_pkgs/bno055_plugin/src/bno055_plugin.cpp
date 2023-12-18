#include "bno055_plugin.hpp"

#define DEBUG true

namespace gazebo
{
    namespace bno055
    {   
        BNO055::BNO055():ModelPlugin() {}

        void BNO055::Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
        {
            this->m_model = model_ptr;
            this->m_ros_node = gazebo_ros::Node::Get(sdf_ptr);
            this->timer = this->m_ros_node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&BNO055::OnUpdate, this));
      			
          	std::string topic_name = "/automobile/IMU";
          	this->m_pubBNO = this->m_ros_node->create_publisher<utils::msg::IMU>(topic_name, 2);

            if(DEBUG)
            {
                auto logger = this->m_ros_node->get_logger();
                std::cerr << "\n\n";
                RCLCPP_INFO_STREAM(logger, "====================================================================");
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] attached to: " << this->m_model->GetName());
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] publish to: "  << topic_name);
                RCLCPP_INFO_STREAM(logger, "[bno055_plugin] Usefull data: linear z, angular x, angular y, angular z");
                RCLCPP_INFO_STREAM(logger, "====================================================================");
            }
        }

        // Publish the updated values
        void BNO055::OnUpdate()
        {        
          
            // TODO: accel fields are missing, is this intentional?
           	this->m_bno055_pose.roll = this->m_model->RelativePose().Rot().Roll();
            this->m_bno055_pose.pitch = this->m_model->RelativePose().Rot().Pitch();
           	this->m_bno055_pose.yaw = this->m_model->RelativePose().Rot().Yaw();
            this->m_pubBNO->publish(this->m_bno055_pose);
        };      
    }; //namespace trafficLight
    GZ_REGISTER_MODEL_PLUGIN(bno055::BNO055)
}; // namespace gazebo
