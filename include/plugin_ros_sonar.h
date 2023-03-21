#ifndef PLUGIN_ROS_SONAR_H
#define PLUGIN_ROS_SONAR_H

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/gazebo.hh"

#include "sensor_msgs/msg/range.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gazebo {
  class RosSonarPlugin : public SensorPlugin
  {
  public:
    RosSonarPlugin() { topicName = "drone/sonar"; }
    virtual ~RosSonarPlugin() {}

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    virtual void onUpdated();

  protected:
    sensors::SonarSensorPtr sonar_;

    event::ConnectionPtr updated_conn_;

    sensor_msgs::msg::Range sonar_msg_;

    std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
    std::string topicName;
  };
}

#endif // PLUGIN_ROS_SONAR_H
