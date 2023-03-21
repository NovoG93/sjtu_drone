#include "plugin_ros_sonar.h"
#include "gazebo/sensors/SonarSensor.hh"

namespace gazebo{
void RosSonarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Make sure the ROS node for Gazebo has already been initialized
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(rclcpp::get_logger("DroneSonarController"), "ROS should be initialized first!");
      return;
    }

    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
    this->sonar_ = std::dynamic_pointer_cast<sensors::SonarSensor>(_sensor);
    if (!this->sonar_){
        gzerr << "SonarPlugin equires a SonarSensor.\n";
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("DroneSonarController"), "The Sonar plugin has been loaded!");

    node_handle_ = std::make_shared<rclcpp::Node>("", "");
    pub_ = node_handle_->create_publisher<sensor_msgs::msg::Range>(topicName, 1);
    this->updated_conn_ = this->sonar_->ConnectUpdated(
                boost::bind(&RosSonarPlugin::onUpdated,this));
}

void RosSonarPlugin::onUpdated(){
    //copy data into ros message

    sonar_msg_.header.frame_id = "drone_link";
    sonar_msg_.header.stamp.sec = this->sonar_->LastUpdateTime().sec;
    sonar_msg_.header.stamp.nanosec = this->sonar_->LastUpdateTime().nsec;

    sonar_msg_.range = this->sonar_->Range();
    sonar_msg_.max_range = this->sonar_->RangeMax();
    sonar_msg_.min_range = this->sonar_->RangeMin();

    pub_->publish(sonar_msg_);
}



GZ_REGISTER_SENSOR_PLUGIN(RosSonarPlugin)
}
