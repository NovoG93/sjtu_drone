// Copyright 2023 Georg Novotny
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.en.html
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "plugin_drone.h"


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo_plugins {

class DroneSimpleControllerPrivate {
public:
  DroneSimpleControllerPrivate()
    : m_timeAfterCmd(0.0)
    , navi_state(LANDED_MODEL)
    , m_posCtrl(false)
    , m_velMode(false)
  {
  }

  ~DroneSimpleControllerPrivate() {}

  void Reset()
  {
    // Reset the values of the controllers
    controllers_.roll.reset();
    controllers_.pitch.reset();
    controllers_.yaw.reset();
    controllers_.velocity_x.reset();
    controllers_.velocity_y.reset();
    controllers_.velocity_z.reset();

    // Set the force and torque acting on the drone to zero
    link->SetForce(ignition::math::Vector3<double>(0.0, 0.0, 0.0));
    link->SetTorque(ignition::math::v6::Vector3<double>(0.0, 0.0, 0.0));

    // Reset the state of the drone
    pose.Reset();
    velocity.Set();
    angular_velocity.Set();
    acceleration.Set();
    euler.Set();
  }

  // ROS Node configuration
  void InitSubscribers(std::string cmd_normal_topic_, std::string posctrl_topic_, std::string imu_topic_, std::string takeoff_topic_, std::string land_topic_, std::string reset_topic_, std::string switch_mode_topic_)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    if (!cmd_normal_topic_.empty()){
      auto sub_opt = create_subscription_options(cmd_normal_topic_, 1);
      cmd_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_normal_topic_, qos, std::bind(&DroneSimpleControllerPrivate::CmdCallback, this, std::placeholders::_1),
        sub_opt);
    }
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No cmd_topic defined!");
    
    if (!posctrl_topic_.empty()) {
      auto sub_opt = create_subscription_options(posctrl_topic_, 1);
      posctrl_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
        posctrl_topic_, qos, std::bind(&DroneSimpleControllerPrivate::PosCtrlCallback, this, std::placeholders::_1),
          sub_opt);
    }
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No position control defined!");

    // subscribe imu
    if (!imu_topic_.empty()) {
      auto sub_opt = create_subscription_options(imu_topic_, 1);
      imu_subscriber_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, std::bind(&DroneSimpleControllerPrivate::ImuCallback, this, std::placeholders::_1),
        sub_opt);
    }
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No imu topic defined!");

    // subscribe command: take off command
    if (!takeoff_topic_.empty()) {
      auto sub_opt = create_subscription_options(takeoff_topic_, 1);
      takeoff_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
        takeoff_topic_, qos, std::bind(&DroneSimpleControllerPrivate::TakeoffCallback, this, std::placeholders::_1),
        sub_opt
      );
    }
    else
        RCLCPP_ERROR(ros_node_->get_logger(), "No takeoff topic defined!");

    // subscribe command: land command
    if (!land_topic_.empty()) {
      auto sub_opt = create_subscription_options(land_topic_, 1);
      land_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
        land_topic_, qos, std::bind(&DroneSimpleControllerPrivate::LandCallback, this, std::placeholders::_1),
        sub_opt);
    }
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No land topic defined!");

    // subscribe command: reset command
    if (!reset_topic_.empty()) {
      auto sub_opt = create_subscription_options(reset_topic_, 1);
      reset_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
        reset_topic_, qos, std::bind(&DroneSimpleControllerPrivate::ResetCallback, this, std::placeholders::_1),
        sub_opt);
    }
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No reset topic defined!");
    
    // Subscribe command: switch mode command
    if (!switch_mode_topic_.empty()) {
      auto sub_opt = create_subscription_options(switch_mode_topic_, 1);
      switch_mode_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
        switch_mode_topic_, qos, std::bind(&DroneSimpleControllerPrivate::SwitchModeCallback, this, std::placeholders::_1),
        sub_opt);
    }
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No switch mode topic defined!");
  }

  void InitPublishers(std::string gt_topic_, std::string gt_vel_topic_, std::string gt_acc_topic_, std::string cmd_mode_topic_, std::string state_topic_)
  {
    if (!gt_topic_.empty())
      pub_gt_pose_ = ros_node_->create_publisher<geometry_msgs::msg::Pose>(gt_topic_,1024);  
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth topic defined!");

    if (!gt_vel_topic_.empty())
      pub_gt_vec_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("gt_vel", 1024);
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth velocity topic defined!");

    if (!gt_acc_topic_.empty())
      pub_gt_acc_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("gt_acc", 1024);
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth acceleration topic defined!");

    if (!cmd_mode_topic_.empty())
      pub_cmd_mode = ros_node_->create_publisher<std_msgs::msg::String>(cmd_mode_topic_, 1024);
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No command mode topic defined!");

    if (!state_topic_.empty())
      pub_state = ros_node_->create_publisher<std_msgs::msg::Int8>(state_topic_, 1024);
    else
      RCLCPP_ERROR(ros_node_->get_logger(), "No state topic defined!");
  }


  // Controller configuration
  /**
  * @brief Initiliaze the PID params
  * 
  * @param _model shared pointer to the model object
  * @param _sdf shared pointer to the sdf object
  */
  void LoadControllerSettings(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf){
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");

    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");

    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Using the PID parameters: \n" <<
                        "\tRoll Pitch:\n" << "\t\tkP: " << controllers_.roll.gain_p << ", kI: " << controllers_.roll.gain_i << ",kD: " << controllers_.roll.gain_d << ", Limit: " << controllers_.roll.limit << ", Time Constant: " << controllers_.roll.time_constant << "\n" << 
                        "\tYaw:\n" << "\t\tkP: " << controllers_.yaw.gain_p << ", kI: " << controllers_.yaw.gain_i << ",kD: " << controllers_.yaw.gain_d << ", Limit: " << controllers_.yaw.limit << ", Time Constant: " << controllers_.yaw.time_constant << "\n" << 
                        "\tVelocity X:\n" << "\t\tkP: " << controllers_.velocity_x.gain_p << ", kI: " << controllers_.velocity_x.gain_i << ",kD: " << controllers_.velocity_x.gain_d << ", Limit: " << controllers_.velocity_x.limit << ", Time Constant: " << controllers_.velocity_x.time_constant << "\n" << 
                        "\tVelocity Y:\n" << "\t\tkP: " << controllers_.velocity_y.gain_p << ", kI: " << controllers_.velocity_y.gain_i << ",kD: " << controllers_.velocity_y.gain_d << ", Limit: " << controllers_.velocity_y.limit << ", Time Constant: " << controllers_.velocity_y.time_constant << "\n" << 
                        "\tVelocity Z:\n" << "\t\tkP: " << controllers_.velocity_z.gain_p << ", kI: " << controllers_.velocity_z.gain_i << ",kD: " << controllers_.velocity_z.gain_d << ", Limit: " << controllers_.velocity_z.limit << ", Time Constant: " << controllers_.velocity_z.time_constant << "\n" << 
                        "\tPosition XY:\n" << "\t\tkP: " << controllers_.pos_x.gain_p << ", kI: " << controllers_.pos_x.gain_i << ",kD: " << controllers_.pos_x.gain_d << ", Limit: " << controllers_.pos_x.limit << ", Time Constant: " << controllers_.pos_x.time_constant << "\n" << 
                        "\tPosition Z:\n" << "\t\tkP: " << controllers_.pos_z.gain_p << ", kI: " << controllers_.pos_z.gain_i << ",kD: " << controllers_.pos_z.gain_d << ", Limit: " << controllers_.pos_z.limit << ", Time Constant: " << controllers_.pos_z.time_constant
    );
  }


  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
    PIDController pos_x;
    PIDController pos_y;
    PIDController pos_z;
  };

  // Gazebo variables
  gazebo::physics::WorldPtr world;
  gazebo::physics::LinkPtr link;

  // Simunlation configuration
  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  // Inertia and mass of the drone
  ignition::math::v6::Vector3<double> inertia;
  double mass;


// private:
//   friend class DroneSimpleController; // Add explicit friend declaration to access private data members

  // Plugin configuration
  double m_timeAfterCmd;
  int navi_state;
  bool m_posCtrl;
  bool m_velMode;

  // rclcpp configuration
  rclcpp::Node::SharedPtr ros_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  // ROS Interfaces
  geometry_msgs::msg::Twist cmd_vel;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr posctrl_subscriber_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_mode_subscriber_{nullptr};

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_gt_pose_{nullptr}; //for publishing ground truth pose
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_vec_{nullptr}; //ground truth velocity in the body frame
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_acc_{nullptr}; //ground truth acceleration in the body frame
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_mode{nullptr}; //for publishing command mode
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_state{nullptr}; //for publishing current STATE (Landed, Flying, Takingoff, Landing)


  // PID Controller
  Controllers controllers_; 

  // State of the drone
  ignition::math::v6::Pose3<double> pose;
  ignition::math::v6::Vector3<double> euler;
  ignition::math::v6::Vector3<double> velocity, acceleration, angular_velocity, position;


  rclcpp::SubscriptionOptions create_subscription_options(std::string topic_name, uint32_t queue_size)
  {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(topic_name + "/statistics");
    return sub_opt;
  }

  // Callbacks
  /**
  * @brief Callback function for the drone command topic.
  * This function is called whenever a new message is received on the drone command topic. It updates
  * the cmd_val member variable with the new command message. It also generates motion noise for the
  * drone's angular and linear velocities by adding drift and small noise values to the command message.
  * The amount of noise added is determined by the motion_drift_noise_time_ and motion_small_noise_
  * member variables of the DroneSimpleController class.
  * The function uses the world->SimTime() function to get the current simulator time and calculate the
  * time difference between the current and last simulation time. The time difference is used to update
  * the drift noise values if the time_counter_for_drift_noise is greater than motion_drift_noise_time_.
  * The updated command message is then used to update the cmd_val member variable.
  * 
  * @param cmd Pointer to the command message containing linear and angular velocities.
  */
  void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
  {
    cmd_vel = *cmd;

    static gazebo::common::Time last_sim_time = world->SimTime();
    static double time_counter_for_drift_noise = 0;
    static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
    // Get simulator time
    gazebo::common::Time cur_sim_time = world->SimTime();
    double dt = (cur_sim_time - last_sim_time).Double();
    // save last time stamp
    last_sim_time = cur_sim_time;

    // generate noise
    if(time_counter_for_drift_noise > motion_drift_noise_time_)
    {
      drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
      drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
      drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
      drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
      time_counter_for_drift_noise = 0.0;
    }
    time_counter_for_drift_noise += dt;

    cmd_vel.angular.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
    cmd_vel.angular.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
    cmd_vel.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
    cmd_vel.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);
  }

  /**
  * @brief Callback function for position control command.
  * This function is called when a new position control command is received.
  * It sets the m_posCtrl flag to the value of the command data.
  * @param cmd The position control command message.
  */
  void PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd)
  {
    m_posCtrl = cmd->data;
  }

  /**
  * @brief Callback function to handle IMU sensor data.
  * @param imu Shared pointer to IMU sensor data.
  * The function reads the quaternion data from the IMU sensor and updates the orientation and angular velocity of the drone.
  */
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
  {
    //directly read the quternion from the IMU data
    pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    euler = pose.Rot().Euler();
    angular_velocity = pose.Rot().RotateVector(ignition::math::v6::Vector3<double>(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
  }

  /**
  * @brief Callback function to initiate taking off of the drone.
  * @param msg Empty message.
  */
  void TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    if(navi_state == LANDED_MODEL)
    {
      navi_state = TAKINGOFF_MODEL;
      m_timeAfterCmd = 0;
      RCLCPP_INFO(ros_node_->get_logger(), "Quadrotor takes off!!");
    }
  }

  /**
  * @brief Callback function to initiate landing of the drone.
  * @param msg Empty message.
  */
  void LandCallback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    if(navi_state == FLYING_MODEL||navi_state == TAKINGOFF_MODEL)
    {
      navi_state = LANDING_MODEL;
      m_timeAfterCmd = 0;
      RCLCPP_INFO(ros_node_->get_logger(), "Quadrotor lands!!");
    }
  }

  /**
  * @brief Callback function for reset command
  * This function resets the controller and the drone's state.
  * @param msg Empty message
  */
  void ResetCallback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Reset quadrotor!!");
    Reset();
  }

  /**
  * @brief Callback function for receiving a message to switch between velocity and position control modes
  * @param msg Shared pointer to the message containing the boolean value for switching the mode
  * The function switches between velocity and position control modes based on the boolean value in the message.
  * If the boolean value is true, the control mode is switched to velocity control and if it's false, the control
  * mode is switched to position control. It also resets the integral term of the controllers for the new mode.
  */
  void SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    m_velMode = msg->data;

    std_msgs::msg::String mode;
    if(m_velMode)
      mode.data = "velocity";
    else
      mode.data = "position";
    pub_cmd_mode->publish(mode);
  }

  /**
  * @brief Updates the current state of the drone.
  * This method is responsible for updating the current state of the drone based on the navigation state.
  * If the drone is taking off, it checks if the time after the command is greater than 0.5 seconds. 
  * If it is, it sets the navigation state to flying. If the drone is landing, it checks if the time after the command is greater than 1 second. 
  * If it is, it sets the navigation state to landed. If the drone is neither taking off nor landing, it resets the time after the command to zero.
  * 
  * @param dt The time elapsed since the last update, in seconds.
  */
  void UpdateState(double dt){
    if(navi_state == TAKINGOFF_MODEL){
      m_timeAfterCmd += dt;
      if (m_timeAfterCmd > 0.5){
          navi_state = FLYING_MODEL;
          std::cout << "Entering flying model!" << std::endl;
      }
    }else if(navi_state == LANDING_MODEL){
      m_timeAfterCmd += dt;
      if(m_timeAfterCmd > 1.0){
          navi_state = LANDED_MODEL;
          std::cout << "Landed!" <<std::endl;
      }
    }else
      m_timeAfterCmd = 0;

    // publish current state using pub_state
    std_msgs::msg::Int8 state_msg;
    state_msg.data = navi_state;
    pub_state->publish(state_msg);
  }


  /**
  * @brief Update the dynamics of the drone.
  * This method updates the dynamics of the drone based on its current state and the current
  * commands being received. It computes the force and torque to be applied to the drone, and
  * updates its position, velocity, and orientation accordingly. It also publishes the ground
  * truth pose, velocity, and acceleration of the drone to ROS topics.
  * 
  * @param dt The time step to use for the update.
  */
  void UpdateDynamics(double dt){
    ignition::math::v6::Vector3<double> force, torque;
    
    // Get Pose/Orientation from Gazebo
    pose = link->WorldPose();
    angular_velocity = link->WorldAngularVel();
    euler = pose.Rot().Euler();
    acceleration = (link->WorldLinearVel() - velocity) / dt;
    velocity = link->WorldLinearVel();
    
    //publish the ground truth pose of the drone to the ROS topic
    geometry_msgs::msg::Pose gt_pose;
    gt_pose.position.x = pose.Pos().X();
    gt_pose.position.y = pose.Pos().Y();
    gt_pose.position.z = pose.Pos().Z();
    
    gt_pose.orientation.w = pose.Rot().W();
    gt_pose.orientation.x = pose.Rot().X();
    gt_pose.orientation.y = pose.Rot().Y();
    gt_pose.orientation.z = pose.Rot().Z();
    pub_gt_pose_->publish(gt_pose);
    
    //convert the acceleration and velocity into the body frame
    ignition::math::v6::Vector3<double> body_vel = pose.Rot().RotateVector(velocity);
    ignition::math::v6::Vector3<double> body_acc = pose.Rot().RotateVector(acceleration);
    
    //publish the velocity
    geometry_msgs::msg::Twist tw;
    tw.linear.x = body_vel.X();
    tw.linear.y = body_vel.Y();
    tw.linear.z = body_vel.Z();
    pub_gt_vec_->publish(tw);
    
    //publish the acceleration
    tw.linear.x = body_acc.X();
    tw.linear.y = body_acc.Y();
    tw.linear.z = body_acc.Z();
    pub_gt_acc_->publish(tw);
    
            
    ignition::math::v6::Vector3<double> poschange = pose.Pos() - position;
    position = pose.Pos();
    
    // Get gravity
    ignition::math::v6::Vector3<double> gravity_body = pose.Rot().RotateVector(world->Gravity());
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body);  // Get gravity

    // Rotate vectors to coordinate frames relevant for control
    ignition::math::v6::Quaternion<double> heading_quaternion(cos(euler[2]/2), 0.0, 0.0, sin(euler[2]/2));
    ignition::math::v6::Vector3<double> velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    ignition::math::v6::Vector3<double> acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::v6::Vector3<double> angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);

    // update controllers
    force.Set(0.0, 0.0, 0.0);
    torque.Set(0.0, 0.0, 0.0);
    
    if( m_posCtrl){
      //position control
      if(navi_state == FLYING_MODEL){
        double vx = controllers_.pos_x.update(cmd_vel.linear.x, position[0], poschange[0], dt);
        double vy = controllers_.pos_y.update(cmd_vel.linear.y, position[1], poschange[1], dt);
        double vz = controllers_.pos_z.update(cmd_vel.linear.z, position[2], poschange[2], dt);

        ignition::math::v6::Vector3<double> vb = heading_quaternion.RotateVectorReverse(ignition::math::v6::Vector3<double>(vx,vy,vz));
        
        double pitch_command =  controllers_.velocity_x.update(vb[0], velocity_xy[0], acceleration_xy[0], dt) / gravity;
        double roll_command  = -controllers_.velocity_y.update(vb[1], velocity_xy[1], acceleration_xy[1], dt) / gravity;
        torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
        torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);            
        force[2]  = mass      * (controllers_.velocity_z.update(vz,  velocity[2], acceleration[2], dt) + load_factor * gravity);
      }
    }else{
      //normal control
      if( navi_state == FLYING_MODEL )
      {
        //hovering
        double pitch_command =  controllers_.velocity_x.update(cmd_vel.linear.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
        double roll_command  = -controllers_.velocity_y.update(cmd_vel.linear.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
        torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
        torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);
        //TODO: add hover by distnace of sonar
      }else{
        //control by velocity
        if( m_velMode){
            double pitch_command =  controllers_.velocity_x.update(cmd_vel.angular.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
            double roll_command  = -controllers_.velocity_y.update(cmd_vel.angular.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
            torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
            torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);              
        }else{
          //control by tilting
          torque[0] = inertia[0] *  controllers_.roll.update(cmd_vel.angular.x, euler[0], angular_velocity_body[0], dt);
          torque[1] = inertia[1] *  controllers_.pitch.update(cmd_vel.angular.y, euler[1], angular_velocity_body[1], dt);
        }
      }
      torque[2] = inertia[2] *  controllers_.yaw.update(cmd_vel.angular.z, angular_velocity[2], 0, dt);
      force[2]  = mass      * (controllers_.velocity_z.update(cmd_vel.linear.z,  velocity[2], acceleration[2], dt) + load_factor * gravity);
    }

    if (max_force_ > 0.0 && force[2] > max_force_) force[2] = max_force_;
    if (force[2] < 0.0) force[2] = 0.0;
    
    // process robot state information
    if(navi_state == LANDED_MODEL)
    {

    }
    else if(navi_state == FLYING_MODEL)
    {
      link->AddRelativeForce(force);
      link->AddRelativeTorque(torque);
    }
    else if(navi_state == TAKINGOFF_MODEL)
    {
      link->AddRelativeForce(force*1.5);
      link->AddRelativeTorque(torque*1.5);
    }
    else if(navi_state == LANDING_MODEL)
    {
      link->AddRelativeForce(force*0.8);
      link->AddRelativeTorque(torque*0.8);
    }
  }
}; // class DroneSimpleControllerPrivate

DroneSimpleController::DroneSimpleController()
: impl_(std::make_unique<DroneSimpleControllerPrivate>())
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DroneSimpleController::~DroneSimpleController()
{
  this->updateConnection.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DroneSimpleController::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf); // TODO: Check if node_name should should be passed to Get(): https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_ros/src/node.cpp#L41C60-L41C69

  if(!rclcpp::ok()){
    RCLCPP_FATAL(impl_->ros_node_->get_logger(), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
    exit(1);
  }
  
  world = _model->GetWorld();
  impl_->world = _model->GetWorld();
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin is loading!"); 
  
  if (!_sdf->HasElement("bodyName"))
  {
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin is loading1!"); 
    impl_->link = _model->GetLink();
    link_name_ = impl_->link->GetName();
  }
  else {
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin is loading2!"); 
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    auto entity = world->EntityByName(link_name_);
    // if (entity) {
      impl_->link = boost::dynamic_pointer_cast<gazebo::physics::Link>(entity);
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin is loading3!");
    // }
    // else {
    //   RCLCPP_FATAL(impl_->ros_node_->get_logger(), "bodyName: %s does not exist\n", link_name_.c_str());
    //   return;
    // }
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "1"); 

  if (!impl_->link)
  {
    RCLCPP_FATAL(impl_->ros_node_->get_logger(), "gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "1"); 

  if (!_sdf->HasElement("maxForce"))
    impl_->max_force_ = -1;
  else
    impl_->max_force_ = _sdf->GetElement("maxForce")->Get<double>();

  if (!_sdf->HasElement("motionSmallNoise"))
    impl_->motion_small_noise_ = 0;
  else
   impl_-> motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoise"))
   impl_->motion_drift_noise_ = 0;
  else
    impl_->motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    impl_->motion_drift_noise_time_ = 1.0;
  else
    impl_->motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();

    

  RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Using following parameters: \n" <<
                      "\t\tlink_name: "<<  link_name_.c_str() << ",\n" <<
                      "\t\tmax_force: "<<  impl_->max_force_ << ",\n" <<
                      "\t\tmotion_small_noise: "<<  impl_->motion_small_noise_ << ",\n" <<
                      "\t\tmotion_drift_noise: "<< impl_-> motion_drift_noise_ << ",\n" <<
                      "\t\tmotion_drift_noise_time: "<<  impl_->motion_drift_noise_time_
                    );

  // get inertia and mass of quadrotor body
  impl_->inertia = impl_->link->GetInertial()->PrincipalMoments();
  impl_->mass = impl_->link->GetInertial()->Mass();


  // Subscriber topics
  //TODO: read from sdf / xacro
  const std::string cmd_normal_topic_ = "cmd_vel";
  const std::string imu_topic_ = "imu";
  const std::string takeoff_topic_ = "takeoff";
  const std::string land_topic_ = "land";
  const std::string reset_topic_ = "reset";
  const std::string posctrl_topic_ = "posctrl";
  const std::string switch_mode_topic_ = "dronevel_mode";

  // Publisher topics
  //TODO: read from sdf / xacro
  const std::string gt_topic_ = "gt_pose";
  const std::string gt_vel_topic_ = "gt_vel";
  const std::string gt_acc_topic_ = "gt_acc";
  const std::string cmd_mode_topic_ = "cmd_mode";
  const std::string state_topic_ = "state";
  
  impl_->InitSubscribers(cmd_normal_topic_, posctrl_topic_, imu_topic_, takeoff_topic_, land_topic_, reset_topic_, switch_mode_topic_);
  impl_->InitPublishers(gt_topic_, gt_vel_topic_, gt_acc_topic_, cmd_mode_topic_, state_topic_);

  impl_->LoadControllerSettings(_model, _sdf);
  
  Reset();

  updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&DroneSimpleController::Update, this));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin finished loading!");
}

/**
* @brief Update method called by the Gazebo simulator every simulation iteration.
*/
void DroneSimpleController::Update()
{ 
  // Get simulator time
  const gazebo::common::Time sim_time = world->SimTime();
  const double dt = (sim_time - last_time).Double();
  if (dt == 0.0) return;
  
  impl_->UpdateState(dt);
  impl_->UpdateDynamics(dt);
  
  // save last time stamp
  last_time = sim_time;   
}

void DroneSimpleController::Reset()
{
  impl_->Reset();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo_plugins