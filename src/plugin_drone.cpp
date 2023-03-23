#include "plugin_drone.h"


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo {

DroneSimpleController::DroneSimpleController()
{ 
  navi_state = LANDED_MODEL;
  m_posCtrl = false;
  m_velMode = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DroneSimpleController::~DroneSimpleController()
{
  this->updateConnection.reset();

  // node_handle_->shutdown();
  // delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  if(!rclcpp::ok()){
    RCLCPP_FATAL(rclcpp::get_logger("DroneSimpleController"), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
  }

  
  world = _model->GetWorld();
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "The drone plugin is loading!");
  
  //default parameters
  model_name_ = _model->GetName().substr(0, _model->GetName().find("::"));
  cmd_normal_topic_ = "cmd_vel";
  takeoff_topic_ = "takeoff";
  land_topic_ = "land";
  reset_topic_ = "reset";
  posctrl_topic_ = "posctrl";
  gt_topic_ = "gt_pose";
  switch_mode_topic_ = "dronevel_mode";
  
  if (!_sdf->HasElement("imuTopic"))
    imu_topic_.clear();
  else
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();
  
  
  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = boost::dynamic_pointer_cast<physics::Link>(world->EntityByName(link_name_));
  }

  if (!link)
  {
    RCLCPP_FATAL(rclcpp::get_logger("DroneSimpleController"), "gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();


  if (!_sdf->HasElement("motionSmallNoise"))
    motion_small_noise_ = 0;
  else
    motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoise"))
    motion_drift_noise_ = 0;
  else
    motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

  if (!_sdf->HasElement("motionDriftNoiseTime"))
    motion_drift_noise_time_ = 1.0;
  else
    motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();

  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "After get param!");

  // get inertia and mass of quadrotor body
  inertia = link->GetInertial()->PrincipalMoments();
  mass = link->GetInertial()->Mass();

  node_handle_ = std::make_shared<rclcpp::Node>("control", model_name_);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // subscribe command: control command
  if (!cmd_normal_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    
    sub_opt.callback_group = node_handle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(cmd_normal_topic_ + "/statistics");

    cmd_subscriber_ = node_handle_->create_subscription<geometry_msgs::msg::Twist>(cmd_normal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&DroneSimpleController::CmdCallback, this, std::placeholders::_1),sub_opt);
    if (cmd_subscriber_->get_topic_name() != "") {
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using cmd_topic %s.", cmd_normal_topic_.c_str());
    }
    else {
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "cannot find the command topic!");
    }
  }
  
  if (!posctrl_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group( rclcpp::CallbackGroupType::Reentrant);

    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(posctrl_topic_ + "/statistics");

    posctrl_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
      posctrl_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::PosCtrlCallback, this, std::placeholders::_1),
        sub_opt);

    if (posctrl_subscriber_->get_topic_name() != "") {
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "find the position control topic: %s!", posctrl_subscriber_->get_topic_name());
    }
    else {
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "cannot find the position control topic!");
    }
  } else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "cannot find the position control topic!: ");

  

  // subscribe imu
  if (!imu_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(imu_topic_ + "/statistics");
    imu_subscriber_ = node_handle_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::ImuCallback, this, std::placeholders::_1),
      sub_opt);

    if (imu_subscriber_->get_topic_name() != "") {
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
    }
    else {
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "cannot find the IMU topic!");
    }
  } else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "cannot find the IMU topic!: ");


  // subscribe command: take off command
  if (!takeoff_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(takeoff_topic_ + "/statistics");

    takeoff_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      takeoff_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::TakeoffCallback, this, std::placeholders::_1),
      sub_opt
    );

    if (takeoff_subscriber_->get_topic_name() != "") {
      RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "find the takeoff topic");
    }
    else {
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "cannot find the takeoff topic!");
    }
  }else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "cannot find the takeoff topic!: ");

  // subscribe command: land command
  if (!land_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(land_topic_ + "/statistics");
    land_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      land_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::LandCallback, this, std::placeholders::_1),
      sub_opt);
  }

  // subscribe command: reset command
  if (!reset_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(reset_topic_ + "/statistics");
    reset_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Empty>(
      reset_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::ResetCallback, this, std::placeholders::_1),
      sub_opt);
  } else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "cannot find the reset topic!: ");
  
  if (!gt_topic_.empty()){
    pub_gt_pose_ = node_handle_->create_publisher<geometry_msgs::msg::Pose>("gt_pose",1024);    
  } else
    pub_gt_pose_ = node_handle_->create_publisher<geometry_msgs::msg::Pose>("gt_pose",1024);    


  
  pub_gt_vec_ = node_handle_->create_publisher<geometry_msgs::msg::Twist>("gt_vel", 1024);
  pub_gt_acc_ = node_handle_->create_publisher<geometry_msgs::msg::Twist>("gt_acc", 1024);
  
  
  if (!switch_mode_topic_.empty()) {
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = node_handle_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    sub_opt.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    sub_opt.topic_stats_options.publish_period = std::chrono::milliseconds(100);
    sub_opt.topic_stats_options.publish_topic = std::string(switch_mode_topic_ + "/statistics");
    switch_mode_subscriber_ = node_handle_->create_subscription<std_msgs::msg::Bool>(
      switch_mode_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&DroneSimpleController::SwitchModeCallback, this, std::placeholders::_1),
      sub_opt);
  } else
      RCLCPP_WARN(rclcpp::get_logger("DroneSimpleController"), "cannot find the switch_mode topic!: ");

      
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "get_sub_namespace: %s", this->node_handle_->get_sub_namespace().c_str());
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "count_subscribers: %i", this->node_handle_->count_subscribers("takeoff"));
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "count_publishers: %i", this->node_handle_->count_publishers("takeoff"));
  std::vector<rclcpp::TopicEndpointInfo> topic_info = this->node_handle_->get_subscriptions_info_by_topic("takeoff");
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Takeoff sub info size: %i", topic_info.size());
  for (int i =0; i < topic_info.size(); i++ ) 
  {
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Takeoff sub info (i)node_name: %s", topic_info[i].node_name().c_str());
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Takeoff sub info (i)topic_type: %s", topic_info[i].topic_type().c_str());
  }
  

  // for (rclcpp::TopicEndpointInfo info: topic_info)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Takeoff sub info: "
  // }
  LoadControllerSettings(_model, _sdf);
  
  Reset();

  executor_->add_node(node_handle_);
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DroneSimpleController::Update, this));

  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "The drone plugin finished!");
}

void DroneSimpleController::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    controllers_.roll.Load(_sdf, "rollpitch");
    controllers_.pitch.Load(_sdf, "rollpitch");
    controllers_.yaw.Load(_sdf, "yaw");
    controllers_.velocity_x.Load(_sdf, "velocityXY");
    controllers_.velocity_y.Load(_sdf, "velocityXY");
    controllers_.velocity_z.Load(_sdf, "velocityZ");
    
    controllers_.pos_x.Load(_sdf, "positionXY");
    controllers_.pos_y.Load(_sdf, "positionXY");
    controllers_.pos_z.Load(_sdf, "positionZ");
    
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void DroneSimpleController::CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  cmd_val = *cmd;


  static common::Time last_sim_time = world->SimTime();
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
  // Get simulator time
  common::Time cur_sim_time = world->SimTime();
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

  cmd_val.angular.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
  cmd_val.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);

}

void DroneSimpleController::PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd){
    m_posCtrl = cmd->data;
}

void DroneSimpleController::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
  //directly read the quternion from the IMU data
  pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(ignition::math::v6::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void DroneSimpleController::TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "In TakeoffCallback ");
  if(navi_state == LANDED_MODEL)
  {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "\nQuadrotor takes off!!");
  }
}

void DroneSimpleController::LandCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if(navi_state == FLYING_MODEL||navi_state == TAKINGOFF_MODEL)
  {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "\nQuadrotor lands!!");
  }

}

void DroneSimpleController::ResetCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "\nReset quadrotor!!");
}

void DroneSimpleController::SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg){
    m_velMode = msg->data;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void DroneSimpleController::Update()
{
    // std::cout << "In DroneSimpleController::Update\n";
    // Get new commands/state
    // callback_group_->callAvailable();
  
    // Get simulator time
    common::Time sim_time = world->SimTime();
    double dt = (sim_time - last_time).Double();
    if (dt == 0.0) return;
    
    executor_->spin_some(std::chrono::milliseconds(100));
    UpdateState(dt);
    UpdateDynamics(dt);
    
    // save last time stamp
    last_time = sim_time;   
}

void DroneSimpleController::UpdateState(double dt){
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
}


void DroneSimpleController::UpdateDynamics(double dt){
  ignition::math::v6::Vector3<double> force, torque;
   
  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  //  if (imu_subscriber_.getTopic()=="")
    {
      pose = link->WorldPose();
      angular_velocity = link->WorldAngularVel();
      euler = pose.Rot().Euler();
    }
   // if (state_topic_.empty())
    {
      acceleration = (link->WorldLinearVel() - velocity) / dt;
      velocity = link->WorldLinearVel();
    }
    
    
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
    ignition::math::v6::Vector3 body_vel = pose.Rot().RotateVector(velocity);
    ignition::math::v6::Vector3 body_acc = pose.Rot().RotateVector(acceleration);
    
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
    
            
    ignition::math::v6::Vector3 poschange = pose.Pos() - position;
    position = pose.Pos();
    
  
    // Get gravity
    ignition::math::v6::Vector3 gravity_body = pose.Rot().RotateVector(world->Gravity());
    double gravity = gravity_body.Length();
    double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body);  // Get gravity
  
    // Rotate vectors to coordinate frames relevant for control
    ignition::math::v6::Quaternion heading_quaternion(cos(euler[2]/2), 0.0, 0.0, sin(euler[2]/2));
    ignition::math::v6::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
    ignition::math::v6::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::v6::Vector3 angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);
  
    // update controllers
    force.Set(0.0, 0.0, 0.0);
    torque.Set(0.0, 0.0, 0.0);
    
    if( m_posCtrl){
        //position control
        if(navi_state == FLYING_MODEL){
            double vx = controllers_.pos_x.update(cmd_val.linear.x, position[0], poschange[0], dt);
            double vy = controllers_.pos_y.update(cmd_val.linear.y, position[1], poschange[1], dt);
            double vz = controllers_.pos_z.update(cmd_val.linear.z, position[2], poschange[2], dt);

            ignition::math::v6::Vector3 vb = heading_quaternion.RotateVectorReverse(ignition::math::v6::Vector3(vx,vy,vz));
            
            double pitch_command =  controllers_.velocity_x.update(vb[0], velocity_xy[0], acceleration_xy[0], dt) / gravity;
            double roll_command  = -controllers_.velocity_y.update(vb[1], velocity_xy[1], acceleration_xy[1], dt) / gravity;
            torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
            torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);            
            force[2]  = mass      * (controllers_.velocity_z.update(vz,  velocity[2], acceleration[2], dt) + load_factor * gravity);
        }
    }else{
        //normal control
        if( navi_state == FLYING_MODEL )//&& cmd_val.linear.x >= 0 && cmd_val.linear.y >= 0)
        {
          //hovering
          double pitch_command =  controllers_.velocity_x.update(cmd_val.linear.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
          double roll_command  = -controllers_.velocity_y.update(cmd_val.linear.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
          torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
          torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);
        }else{
          //control by velocity
          if( m_velMode){
              double pitch_command =  controllers_.velocity_x.update(cmd_val.angular.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
              double roll_command  = -controllers_.velocity_y.update(cmd_val.angular.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
              torque[0] = inertia[0] *  controllers_.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
              torque[1] = inertia[1] *  controllers_.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);              
          }else{
            //control by tilting
            torque[0] = inertia[0] *  controllers_.roll.update(cmd_val.angular.x, euler[0], angular_velocity_body[0], dt);
            torque[1] = inertia[1] *  controllers_.pitch.update(cmd_val.angular.y, euler[1], angular_velocity_body[1], dt);
          }
        }
        torque[2] = inertia[2] *  controllers_.yaw.update(cmd_val.angular.z, angular_velocity[2], 0, dt);
        force[2]  = mass      * (controllers_.velocity_z.update(cmd_val.linear.z,  velocity[2], acceleration[2], dt) + load_factor * gravity);
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
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void DroneSimpleController::Reset()
{
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  link->SetForce(ignition::math::Vector3(0.0, 0.0, 0.0));
  link->SetTorque(ignition::math::v6::Vector3(0.0, 0.0, 0.0));

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp_ = rclcpp::Time();

}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo
