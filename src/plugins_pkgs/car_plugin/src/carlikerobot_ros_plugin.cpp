#include "carlikerobot_ros_plugin.hpp"

using std::placeholders::_1;

#define DEBUG true

namespace gazebo
{
    namespace carlikerobot
    {
        /// Movement serialHandler
        CMessageHandler::CMessageHandler(rclcpp::Node::SharedPtr rosNode, IRobotCommandSetter* _setter)
        {
            this->_robotSetter = _setter;
            this->_rosNode = rosNode;

            // Generate topic name
            std::string topicName = "/automobile/command";
            std::string listen_topicName = "/automobile/feedback";

            auto logger = this->_rosNode->get_logger();

            this->_commandSubscriber = this->_rosNode->create_subscription<std_msgs::msg::String>(topicName, 10, std::bind(&CMessageHandler::OnMsgCommand, this, std::placeholders::_1));

            // Feedback message
			this->_feedbackPublisher = this->_rosNode->create_publisher<std_msgs::msg::String>(listen_topicName, 2);
		
            if (DEBUG)
            {
                auto logger = this->_rosNode->get_logger();
                std::cerr << "\n\n";
                RCLCPP_INFO_STREAM(logger, "====================================================================");
                RCLCPP_INFO_STREAM(logger, "publish to: "  << topicName);
                RCLCPP_INFO_STREAM(logger, "listen to: "  << listen_topicName);
                RCLCPP_INFO_STREAM(logger, "Usefull data: /Response for ack, /Command for speed and steer");
                RCLCPP_INFO_STREAM(logger, "====================================================================");
            }
        }

        CMessageHandler::~CMessageHandler()
        {
        }

        // Callback function for receiving messages on the /car_name/Command topic
        void CMessageHandler::OnMsgCommand(const std_msgs::msg::String::SharedPtr _msg)
        {           
        	rapidjson::Document doc;
            auto logger = this->_rosNode->get_logger();
        	const char* c = _msg->data.c_str();
        	doc.Parse(c);
        	if (doc.HasMember("action"))
		    {
			    std::string command = doc["action"].GetString();
		    	if(command =="1")
		    	{
		    		if (DEBUG){RCLCPP_INFO_STREAM(logger, "Received SPED message");}
		    		if (doc.HasMember("speed")){ this->spedMessage(doc["speed"].GetFloat());}
		    		else{RCLCPP_INFO_STREAM(logger, "Invalid message"); this->unknownMessage();}
		    	} else if (command =="2") {
				    if (DEBUG){RCLCPP_INFO_STREAM(logger, "Received STER message");}
				    if (doc.HasMember("steerAngle")){ this->sterMessage(doc["steerAngle"].GetFloat());}
		    		else{RCLCPP_INFO_STREAM(logger, "Invalid message"); this->unknownMessage();}
		    	} else if (command =="3") {
		            if (DEBUG){RCLCPP_INFO_STREAM(logger, "Received BRAKE message");}
		            if (doc.HasMember("steerAngle")){ this->brakeMessage(doc["steerAngle"].GetFloat());}
		    		else{RCLCPP_INFO_STREAM(logger, "Invalid message"); this->unknownMessage();}
		    	} else {
		            RCLCPP_INFO_STREAM(logger, "Received UNKNOWN message");
		            this->unknownMessage();
		    	}
	    	} else {
	    		RCLCPP_INFO_STREAM(logger, "Invalid message");
	    		this->unknownMessage();
	    	}
        }

        void CMessageHandler::unknownMessage()
        {
            std_msgs::msg::String l_resp;
            l_resp.data = "@MESS:err;;";
            this->_feedbackPublisher->publish(l_resp);
        }

        void CMessageHandler::brakeMessage(float _msg_val)
        {
            _robotSetter->f_speed = 0;
            _robotSetter->f_steer = _msg_val;
            _robotSetter->setCommand();
            std_msgs::msg::String l_resp;
            l_resp.data= "@3:ack;;";
            this->_feedbackPublisher->publish(l_resp);
        }

        void CMessageHandler::spedMessage(float _msg_val)
        {
            _robotSetter->f_speed = _msg_val;
            _robotSetter->setCommand();
            std_msgs::msg::String l_resp;
            l_resp.data = "@1:ack;;";
            this->_feedbackPublisher->publish(l_resp);
        }

        void CMessageHandler::sterMessage(float _msg_val)
        {
            _robotSetter->f_steer = _msg_val;
            _robotSetter->setCommand();
            std_msgs::msg::String l_resp;
            l_resp.data = "@2:ack;;";
            this->_feedbackPublisher->publish(l_resp);
        }

        /// end movement serialHandler.
	    

        bool CCarLikeRobotRosPlugin::LoadParameterJoints(sdf::ElementPtr f_sdf)
        {
            // start [wheelbase] [axletrack] [wheelradius] handling*/
            double l_wheelbase   = 0;
            double l_axletrack   = 0;  
            double l_wheelradius = 0; 
            auto logger = rclcpp::get_logger("CarLikeRobotPlugin");
            
            if(DEBUG)
            {
                std::cerr << "\n\n";
                RCLCPP_INFO_STREAM(logger, "====================================================================");
            }

            if(f_sdf->HasElement("wheelbase"))
            {
                l_wheelbase = f_sdf->Get<double>("wheelbase");

                if(DEBUG)
                {
                    RCLCPP_INFO_STREAM(logger, "OK [wheelbase]   = " << l_wheelbase);
                }
            }
            else
            {
                if(DEBUG)
                {
                    RCLCPP_INFO_STREAM(logger, "WARNING: [wheelbase] = 0 DEFAULT");
                }
            }
            
            if(f_sdf->HasElement("axletrack"))
            {
                l_axletrack = f_sdf->Get<double>("axletrack");

                if(DEBUG)
                {
                    RCLCPP_INFO_STREAM(logger, "OK [axletrack]   = " << l_axletrack);
                }
            }
            else
            {
                RCLCPP_INFO_STREAM(logger, "WARNING: [axletrack] = 0 DEFAULT");
            }
            
            if(f_sdf->HasElement("wheelradius"))
            {
                l_wheelradius = f_sdf->Get<double>("wheelradius");

                if(DEBUG)
                {
                    RCLCPP_INFO_STREAM(logger, "OK [wheelradius] = " << l_wheelradius);
                }
            }
            else
            {
                std::cerr << "WARNING: [wheelradius] = 0 DEFAULT\n";
                std::cerr << "CRITICAL: Invalid plugin parameters, wrong wheel radius.\n\
                              CarLikeRobotPlugin plugin is not loaded.\n";
                return false;
            }
            // end [wheelbase] [axletrack] [wheelradius] handling*/
            
            // start [speed_wheel_joints] [front_wheel_joints] [rear_wheel_joints]
            sdf::ElementPtr l_steering_joints_sdf   = NULL;
            sdf::ElementPtr l_speed_wheel_sdf       = NULL; 
            sdf::ElementPtr l_front_wheel_sdf       = NULL;
            sdf::ElementPtr l_rear_wheel_sdf        = NULL;
            
            if(f_sdf->HasElement("speed_wheel_joints"))
            {
    
                double l_kp_speed = 0;
                double l_ki_speed = 0;
                double l_kd_speed = 0;
    
                l_speed_wheel_sdf = f_sdf->GetElement("speed_wheel_joints");

                if(DEBUG)
                {
                    RCLCPP_INFO_STREAM(logger, "====================================================================");
                    std::cerr << "\n\n";
                    RCLCPP_INFO_STREAM(logger, "====================================================================");
                    RCLCPP_INFO_STREAM(logger, "FOUND: [speed_wheel_joints]");
                }

                // start [kp] [ki] [kd] 
                if(l_speed_wheel_sdf->HasElement("kp"))
                {
                    l_kp_speed = l_speed_wheel_sdf->Get<double>("kp");
                    
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "OK [kp] = " << l_kp_speed);
                    } 
                }
                else
                {
                    std::cerr << "<kp> under <speed_wheel_joints> not found. 0 DEFAULT\n";
                }

                if(l_speed_wheel_sdf->HasElement("kd"))
                {
                    l_kd_speed = l_speed_wheel_sdf->Get<double>("kd");
                    
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "OK [kd] = " << l_kd_speed);
                    }
                }
                else
                {
                    std::cerr << "<kd> under <speed_wheel_joints> not found. 0 DEFAULT\n";
                }

                if(l_speed_wheel_sdf->HasElement("ki"))
                {
                    l_ki_speed = l_speed_wheel_sdf->Get<double>("ki");
                    
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "OK [ki] = " << l_ki_speed);
                    }
                }
                else
                {
                    std::cerr << "<ki> under <speed_wheel_joints> not found. 0 DEFAULT\n";
                }// end [kp] [ki] [kd] 


                // start HasElement("front_wheel_joints")
                if(l_speed_wheel_sdf->HasElement("front_wheel_joints"))
                {
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "====================================================================");
                        std::cerr << "\n\n";
                        RCLCPP_INFO_STREAM(logger, "====================================================================");
                        RCLCPP_INFO_STREAM(logger, "FOUND: [front_wheel_joints]");
                    }

                    l_front_wheel_sdf = l_speed_wheel_sdf->GetElement("front_wheel_joints");

                    // START FRONT WHEELS CONTROLLER FOR SPINNING
                    std::string l_left  = l_front_wheel_sdf->Get<std::string>("leftjoint");
                    if(l_left == " ")
                    {
                        std::cerr << "CRITICAL: empty front [leftjoint] name. Plugin WAS NOT LOADED. exitting...\n";
                        return false;
                    }
                    else
                    {
                        if(DEBUG)
                        {
                            RCLCPP_INFO_STREAM(logger, "OK front [leftjoint]  name = " << l_left.c_str());
                        }
                    }

                    std::string l_right = l_front_wheel_sdf->Get<std::string>("rightjoint");
                    if(l_right == " ")
                    {
                        std::cerr << "CRITICAL: empty front [rightjoint] name. Plugin WAS NOT LOADED. exitting...\n";
                        return false;
                    }
                    else
                    {
                        if(DEBUG)
                        {
                            RCLCPP_INFO_STREAM(logger, "OK front [rightjoint] name = " << l_right.c_str());
                        }
                    }

                    physics::JointPtr l_rightJoint  = this->_model->GetJoint(l_right);
                    if(l_rightJoint == NULL)
                    {
                        std::cerr << "CRITICAL: front [rightjoint] name MISMACH. Check model's joints names\
                                      Plugin WAS NOT LOADED. exitting...\n";
                        return false;
                    }
                    else
                    {
                        if(DEBUG)
                        {
                            RCLCPP_INFO_STREAM(logger, "OK front [rightjoint] was found in model");
                        }
                    }

                    physics::JointPtr l_leftJoint   = this->_model->GetJoint(l_left);
                    if(l_leftJoint == NULL)
                    {
                        std::cerr << "CRITICAL: front [leftjoint] name MISMACH. Check model's joints names\
                                      Plugin WAS NOT LOADED. exitting...\n";
                        return false;
                    }
                    else
                    {
                        if(DEBUG)
                        {
                            RCLCPP_INFO_STREAM(logger, "OK front [leftjoint]  was found in model");
                        }
                    }

                    // PID
                    common::PID l_rightPID  = common::PID(l_kp_speed, l_ki_speed, l_kd_speed);
                    common::PID l_leftPID   = common::PID(l_kp_speed, l_ki_speed, l_kd_speed);

                    // FrontWheelsSpeed
                    this->_frontWheelSpeedPtr = IWheelsSpeedPtr(new CFrontWheelsSpeed(l_axletrack,
                                                                                      l_wheelbase,
                                                                                      l_wheelradius,
                                                                                      l_rightJoint,
                                                                                      l_leftJoint,
                                                                                      l_rightPID,
                                                                                      l_leftPID,
                                                                                      this->_model));
                    // END FRONT WHEELS CONTROLLER FOR SPINNING
                }// END HasElement("front_wheel_joints")


                // start HasElement("rear_wheel_joints") 
                if(l_speed_wheel_sdf->HasElement("rear_wheel_joints"))
                {
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "====================================================================");
                        std::cerr << "\n\n";
                        RCLCPP_INFO_STREAM(logger, "====================================================================");
                        RCLCPP_INFO_STREAM(logger, "FOUND: [rear_wheel_joints]");
                    }

                    l_rear_wheel_sdf = l_speed_wheel_sdf->GetElement("rear_wheel_joints");

                    // START REAR WHEELS CONTROLLER FOR SPINNING
                    std::string l_left  = l_rear_wheel_sdf->Get<std::string>("leftjoint");
                    if(l_left == " ")
                    {
                        std::cerr << "CRITICAL: empty rear [leftjoint] name. Plugin WAS NOT LOADED. exitting...\n";
                        return false;
                    }
                    else
                    {
                        if(DEBUG)
                        {
                            RCLCPP_INFO_STREAM(logger, "OK rear [leftjoint]  name = " << l_left.c_str());
                        }
                    }

                    std::string l_right = l_rear_wheel_sdf->Get<std::string>("rightjoint");
                    if(l_right == " ")
                    {
                        std::cerr << "CRITICAL: empty rear [rightjoint] name. Plugin WAS NOT LOADED. exitting...\n";
                        return false;
                    }
                    else
                    {
                       if(DEBUG)
                       {
                            RCLCPP_INFO_STREAM(logger, "OK rear [rightjoint] name = " << l_right.c_str());
                       }
                    }

                    physics::JointPtr l_rightJoint  = this->_model->GetJoint(l_right);
                    if(l_rightJoint == NULL)
                    {
                        std::cerr << "CRITICAL: rear [rightjoint] name MISMACH. Check model's joints names\
                                      Plugin WAS NOT LOADED. exitting...\n";
                        return false; 
                    }
                    else
                    {
                        if(DEBUG)
                        {
                            RCLCPP_INFO_STREAM(logger, "OK rear [rightjoint] was found in model");
                        }
                    }

                    physics::JointPtr l_leftJoint   = this->_model->GetJoint(l_left);
                    if(l_leftJoint == NULL)
                    {
                        std::cerr << "CRITICAL: rear [leftjoint] name MISMACH. Check model's joints names\
                                      Plugin WAS NOT LOADED. exitting...\n";
                        return false;
                    }
                    else
                    {
                        if(DEBUG)
                        {
                            RCLCPP_INFO_STREAM(logger, "OK rear [leftjoint] was found in model");
                        }
                    }

                    // PID
                    common::PID l_rightPID = common::PID(l_kp_speed,l_ki_speed,l_kd_speed);
                    common::PID l_leftPID  = common::PID(l_kp_speed,l_ki_speed,l_kd_speed);
                    
                    // RearWheelsSpeed
                    this->_rearWheelsSpeedPtr = IWheelsSpeedPtr(new CRearWheelsSpeed( l_axletrack
                                                                    ,l_wheelbase
                                                                    ,l_wheelradius
                                                                    ,l_rightJoint
                                                                    ,l_leftJoint
                                                                    ,l_rightPID
                                                                    ,l_leftPID
                                                                    ,this->_model));
                    // END REAR WHEELS CONTROLLER FOR SPINNING
                }// END HasElement("rear_wheel_joints")                
            
            }// END [speed_wheel_joints] [front_wheel_joints] [rear_wheel_joints]
            

            if(f_sdf->HasElement("steer_wheel_joints"))
            {

                l_steering_joints_sdf = f_sdf->GetElement("steer_wheel_joints");
                
                if(DEBUG)
                {
                    RCLCPP_INFO_STREAM(logger, "====================================================================");
                    std::cerr << "\n\n";
                    RCLCPP_INFO_STREAM(logger, "====================================================================");    
                    RCLCPP_INFO_STREAM(logger, "FOUND: [steer_wheel_joints]");
                }
                
                double l_kp_position = 0;
                double l_ki_position = 0;
                double l_kd_position = 0;

                // start [kp] [ki] [kd] 
                if(l_steering_joints_sdf->HasElement("kp"))
                {
                    l_kp_position = l_steering_joints_sdf->Get<double>("kp");
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "OK [kp] = " << l_kp_position);
                    }
                }
                else
                {
                    std::cerr << "<kp> under <speed_wheel_joints> not found. 0 DEFAULT\n";
                }

                if(l_steering_joints_sdf->HasElement("kd"))
                {
                    l_kd_position = l_steering_joints_sdf->Get<double>("kd");
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "OK [kd] = " << l_kd_position);
                    }
                }
                else
                {
                    std::cerr << "<kd> under <speed_wheel_joints> not found. 0 DEFAULT\n";
                }

                if(l_steering_joints_sdf->HasElement("ki"))
                {
                    l_ki_position = l_steering_joints_sdf->Get<double>("ki");
                    
                    if(DEBUG)
                    {
                        RCLCPP_INFO_STREAM(logger, "OK [ki] = " << l_ki_position);
                    }
                }
                else
                {
                    std::cerr << "<ki> under <speed_wheel_joints> not found. 0 DEFAULT\n";
                }// end [kp] [ki] [kd] 
                

                // Steering angle pid
                std::string l_left  = l_steering_joints_sdf->Get<std::string>("leftjoint");
                std::string l_right = l_steering_joints_sdf->Get<std::string>("rightjoint");
                
                if(l_left=="" || l_right==""){
                    std::cerr << "CRITICAL: Invalid steering joints. CarLikeRobotPlugin plugin is not loaded.\n";
                    return false;
                }


                physics::JointPtr l_rightJoint = this->_model->GetJoint(l_right);
                physics::JointPtr l_leftJoint  = this->_model->GetJoint(l_left);
                
                if(l_leftJoint==NULL || l_rightJoint==NULL){
                    std::cerr<<"Invalid steering joints. CarLikeRobotPlugin plugin is not loaded.\n";
                    return false;
                }

                // PID
                common::PID l_rightPID = common::PID(l_kp_position,l_ki_position,l_kd_position);
                common::PID l_leftPID  = common::PID(l_kp_position,l_ki_position,l_kd_position);
                // A steering angle calculator
                this->_steerWheelsAnglePtr = ISteerWheelsPtr(new CSteerWheelsAngle( l_axletrack
                                                                    ,l_wheelbase
                                                                    ,l_rightJoint
                                                                    ,l_leftJoint
                                                                    ,l_rightPID
                                                                    ,l_leftPID
                                                                    ,this->_model));
                // END HasElement("steer_wheel_joints")
            }

            return true;
        }


        void CCarLikeRobotRosPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Safety check
            if (_parent->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, CarLikeRobotPlugin plugin not loaded\n";
                return;
            }

            this->_model = _parent;
            bool isLoaded = this->LoadParameterJoints(_sdf);
            auto rosNode = gazebo_ros::Node::Get(_sdf);

            if(!isLoaded){
                return;
            }

            this->f_steer = _sdf->Get<float>("initial_steer");
            this->f_speed = _sdf->Get<float>("initial_speed");

            this->setCommand();
            
            _messageHandler = std::shared_ptr<CMessageHandler>(new CMessageHandler(rosNode,this));
        }

        void CCarLikeRobotRosPlugin::setCommand(){
            if(this->_rearWheelsSpeedPtr!=NULL){
               this->_rearWheelsSpeedPtr->update(this->f_steer,this->f_speed); 
            }
            
            if(this->_frontWheelSpeedPtr!=NULL){
                this->_frontWheelSpeedPtr->update(this->f_steer,this->f_speed);    
            }

            if(this->_steerWheelsAnglePtr!=NULL){
                this->_steerWheelsAnglePtr->update(this->f_steer);               
            }
            
        }
                
    }; // namespace CarLikeRobot
    GZ_REGISTER_MODEL_PLUGIN(carlikerobot::CCarLikeRobotRosPlugin)
} // namespace gazebo
