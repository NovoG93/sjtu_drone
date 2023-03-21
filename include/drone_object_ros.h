#ifndef ARDRONE_ROS_H
#define ARDRONE_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>


/**
 * @brief A simple class to send the commands to the drone through 
 * the corresponding topics
 */

class DroneObjectROS{
protected:
    DroneObjectROS(){}
public:
    
    DroneObjectROS(std::shared_ptr<rclcpp::Node> node){
        initROSVars(node);
    }

    bool isFlying;
    bool isPosctrl;
    bool isVelMode;
    
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubLand;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubReset;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmd;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubPosCtrl;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubVelMode;
    
    std_msgs::msg::Empty empty_msg;
    geometry_msgs::msg::Twist twist_msg;
    
    void initROSVars(std::shared_ptr<rclcpp::Node>);
    
    bool takeOff();
    bool land();
    bool hover();
    bool posCtrl(bool on);
    bool velMode(bool on);
    
    // commands for controling ARDrone
    // pitch_lr = left-right tilt		(-1) to right		(+1)
    // roll_fb = front-back tilt 		(-1) to backwards	(+1)
    // v_du = velocity downwards	(-1) to upwards		(+1)
    // w_lr = angular velocity left (-1) to right		(+1)

    bool move(float v_lr, float v_fb, float v_du, float w_lr);
    bool moveTo(float x, float y, float z);
    bool pitch(float speed = 0.2);
    bool roll(float speed = 0.2);
    bool rise(float speed = 0.1);
    bool yaw(float speed = 0.1);
};

#endif // ARDRONE_ROS_H
