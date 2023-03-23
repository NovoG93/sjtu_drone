#include "plugin_ros_sonar.h"
#include "gazebo/sensors/SonarSensor.hh"

namespace gazebo{
void RosSonarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
    // Make sure the ROS node for Gazebo has already been initialized
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(rclcpp::get_logger("DroneSonarController"), "ROS should be initialized first!");
      return;
    }
    //TODO: find better solution to get model ns
    model_name_ = _sensor->ParentName().substr(0, _sensor->ParentName().find("::"));

    if (!_sdf->HasElement("sonarTopic"))
        topic_name_ = "sonar";
    else
        topic_name_ = _sdf->GetElement("imuTopic")->Get<std::string>();

    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
    this->sonar_ = std::dynamic_pointer_cast<sensors::SonarSensor>(_sensor);
    if (!this->sonar_){
        gzerr << "SonarPlugin equires a SonarSensor.\n";
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("DroneSonarController"), "The Sonar plugin has been loaded!\nRadius: %f", sonar_->Radius());

    node_handle_ = std::make_shared<rclcpp::Node>(topic_name_, model_name_);
    pub_ = node_handle_->create_publisher<sensor_msgs::msg::Range>(topic_name_, 1);
    this->updated_conn_ = this->sonar_->ConnectUpdated(
                std::bind(&RosSonarPlugin::onUpdated,this));
}

void RosSonarPlugin::onUpdated(){
    //copy data into ros message
    sonar_msg_.header.frame_id = "drone_link";
    sonar_msg_.header.stamp.sec = this->sonar_->LastUpdateTime().sec;
    sonar_msg_.header.stamp.nanosec = this->sonar_->LastUpdateTime().nsec;
    
    sonar_msg_.range = this->sonar_->Range();
    sonar_msg_.max_range = this->sonar_->RangeMax();
    sonar_msg_.min_range = this->sonar_->RangeMin();
    sonar_msg_.field_of_view = this->sonar_->Radius();
    sonar_msg_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    sonar_msg_.header.stamp = rclcpp::Clock().now();

    pub_->publish(sonar_msg_);
}



GZ_REGISTER_SENSOR_PLUGIN(RosSonarPlugin)
}
