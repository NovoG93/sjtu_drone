#include "plugin_ros_sonar.h"
//#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/SonarSensor.hh"
namespace gazebo{
void RosSonarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_INFO("ROS should be initialized first!");
      return;
    }

    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
    //this->imu_ = boost::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
    this->sonar_ = std::dynamic_pointer_cast<sensors::SonarSensor>(_sensor);
    //if (!this->imu_){
    if (!this->sonar_){
        gzerr << "SonarPlugin equires a SonarSensor.\n";
        return;
    }

    ROS_INFO("The Sonar plugin has been loaded!");

    node_handle_ = new ros::NodeHandle("");
    //pub_ = node_handle_->advertise<sensor_msgs::Imu>(topicName, 1);
    pub_ = node_handle_->advertise<sensor_msgs::Range>(topicName, 1);
    //this->updated_conn_ = this->imu_->ConnectUpdated(
    //            boost::bind(&RosImuPlugin::onUpdated, this));
    this->updated_conn_ = this->sonar_->ConnectUpdated(
                boost::bind(&RosSonarPlugin::onUpdated,this));
}

/*void RosSonarPlugin::onUpdated(){
   msgs::IMU imu_msg =   this->imu_->ImuMessage();

   //copy data into ros message
   imu_msg_.header.frame_id = "drone_link";
   imu_msg_.header.stamp.sec = imu_msg.stamp().sec();
   imu_msg_.header.stamp.nsec = imu_msg.stamp().nsec();

   imu_msg_.orientation.x = imu_msg.orientation().x();
   imu_msg_.orientation.y = imu_msg.orientation().y();
   imu_msg_.orientation.z = imu_msg.orientation().z();
   imu_msg_.orientation.w = imu_msg.orientation().w();

   // pass angular rates
   imu_msg_.angular_velocity.x    = imu_msg.angular_velocity().x();
   imu_msg_.angular_velocity.y    = imu_msg.angular_velocity().y();
   imu_msg_.angular_velocity.z    = imu_msg.angular_velocity().z();

   imu_msg_.linear_acceleration.x = imu_msg.linear_acceleration().x();
   imu_msg_.linear_acceleration.y = imu_msg.linear_acceleration().y();
   imu_msg_.linear_acceleration.z = imu_msg.linear_acceleration().z();

   pub_.publish(imu_msg_);

}*/
void RosSonarPlugin::onUpdated(){
    //copy data into ros message

    sonar_msg_.header.frame_id = "drone_link";
    sonar_msg_.header.stamp.sec = this->sonar_->LastUpdateTime().sec;
    sonar_msg_.header.stamp.nsec = this->sonar_->LastUpdateTime().nsec;

    sonar_msg_.range = this->sonar_->Range();
    sonar_msg_.max_range = this->sonar_->RangeMax();
    sonar_msg_.min_range = this->sonar_->RangeMin();

    pub_.publish(sonar_msg_);
}



GZ_REGISTER_SENSOR_PLUGIN(RosSonarPlugin)
}
