#include "plugin_ros_imu_native.h"
#include "gazebo/sensors/ImuSensor.hh"
namespace gazebo{
void RosImuPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_INFO("ROS should be initialized first!");
      return;
    }
    
    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
        
    /** Changed by TC to allow C++1 compilation
    this->imu_ = boost::dynamic_pointer_cast<sensors::ImuSensor>(_sensor); **/
    this->imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);    
    if (!this->imu_){
        gzerr << "ImuPlugin equires a ImuSensor.\n";
        return;
    }
    
    ROS_INFO("The IMU plugin has been loaded!");
    
    node_handle_ = new ros::NodeHandle("");
    pub_ = node_handle_->advertise<sensor_msgs::Imu>(topicName, 1);

    this->updated_conn_ = this->imu_->ConnectUpdated(
                boost::bind(&RosImuPlugin::onUpdated, this));
}

void RosImuPlugin::onUpdated(){
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
   
}

GZ_REGISTER_SENSOR_PLUGIN(RosImuPlugin)
}