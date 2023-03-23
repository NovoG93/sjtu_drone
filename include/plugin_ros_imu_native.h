#ifndef PLUGIN_ROS_IMU_NATIVE_H
#define PLUGIN_ROS_IMU_NATIVE_H

#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace gazebo {

class RosImuPlugin : public gazebo::SensorPlugin
{
public:
  RosImuPlugin();
  virtual ~RosImuPlugin();

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;
  void onUpdate();

private:
  gazebo::sensors::ImuSensorPtr imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Node::SharedPtr node_handle_;

  sensor_msgs::msg::Imu imu_msg_;

  std::string model_name_;
  std::string topic_name_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;


};

} // namespace gazebo
#endif // PLUGIN_ROS_IMU_NATIVE_H
