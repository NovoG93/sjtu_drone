#include "drone_object_ros.h"

void DroneObjectROS::initROSVars(rclcpp::Node::SharedPtr node){
    isFlying = false;
    isPosctrl = false;
    isVelMode = false;
    pubTakeOff = node->create_publisher<std_msgs::msg::Empty>("/drone/takeoff", 1024);
    pubLand = node->create_publisher<std_msgs::msg::Empty>("/drone/land", 1024);
    pubReset = node->create_publisher<std_msgs::msg::Empty>("/drone/reset", 1024);
    pubPosCtrl = node->create_publisher<std_msgs::msg::msg::Bool>("/drone/posctrl", 1024);
    pubCmd = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1024);
    pubVelMode = node->create_publisher<std_msgs::msg::msg::Bool>("/drone/vel_mode", 1024);
    const auto qos_sensor_data = rclcpp::SensorDataQoS(rclcpp::KeepLast(10));

}

bool DroneObjectROS::takeOff(){
    if( isFlying)
        return false;
    pubTakeOff->publish(empty_msg);
    // ROS_INFO("Taking Off...");
    
    isFlying = true;
    return true;    
}

bool DroneObjectROS::land(){
    if( !isFlying)
        return false;
    
    pubLand->publish(empty_msg);
    // ROS_INFO("Landing...");
    
    isFlying = false;
    return true;
}


bool DroneObjectROS::hover(){
    if( !isFlying)
        return false;
    
    twist_msg.linear.x=0;
    twist_msg.linear.y=0;
    twist_msg.linear.z=0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= 0.0;
    
    pubCmd->publish(twist_msg);
    // ROS_INFO("Hovering...");
    return true;
}

bool DroneObjectROS::posCtrl(bool on){
    if( !isFlying)
        return false;

    isPosctrl = on;
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = on ? 1:0;
        
    pubPosCtrl->publish(bool_msg);
    
    // if( on)
    //     ROS_INFO("Switching position control on...");
    // else
    //     ROS_INFO("Switching position control off...");
    
    return true;    
}

bool DroneObjectROS::velMode(bool on){
    if (! isFlying)
        return false;
    
    isVelMode = on;
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = on ? 1:0;
        
    pubVelMode->publish(bool_msg);
    
    // if( on)
    //     ROS_INFO("Switching velocity mode on...");
    // else
    //     ROS_INFO("Switching velocity mode off...");
    
    return true;    
}




bool DroneObjectROS::move(float lr, float fb, float ud, float w) {
    if (!isFlying)
        return false;
    
    twist_msg.linear.x=1.0;
    twist_msg.linear.y=1.0;
    twist_msg.linear.z=ud;
    twist_msg.angular.x=lr;
    twist_msg.angular.y=fb;
    twist_msg.angular.z= w;
    pubCmd->publish(twist_msg);
    return true;
}

bool DroneObjectROS::moveTo(float x, float y, float z){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x=x;
    twist_msg.linear.y=y;
    twist_msg.linear.z=z;
    twist_msg.angular.x=0;
    twist_msg.angular.y=0;
    twist_msg.angular.z= 0;
    
    pubCmd->publish(twist_msg);
}

bool DroneObjectROS::pitch(float speed){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x = 1.0;
    twist_msg.linear.y= 1.0;
    twist_msg.linear.z= 0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=speed;
    twist_msg.angular.z= 0.0;
    pubCmd->publish(twist_msg);
    return true;
}
bool DroneObjectROS::roll(float speed){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x = 1.0;
    twist_msg.linear.y= 1.0;
    twist_msg.linear.z= 0;
    twist_msg.angular.x=speed;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= 0.0;
    pubCmd->publish(twist_msg);
    return true;
}

bool DroneObjectROS::rise(float speed){
    if (!isFlying)
        return false;
    
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = speed;
    twist_msg.angular.x = 0.0;//flag for preventing hovering
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    pubCmd->publish(twist_msg);
    return true;
}

bool DroneObjectROS::yaw(float speed){
    if (!isFlying)
        return true;
    
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y= 0.0;
    twist_msg.linear.z= 0;
    twist_msg.angular.x=0.0;//flag for preventing hovering
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= speed;
    pubCmd->publish(twist_msg);
    return true;
}
