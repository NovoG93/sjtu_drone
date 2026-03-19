// Copyright 2024 Georg Novotny
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

#include "sjtu_drone_description/plugin_drone_private.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

#include <gz/sim/Link.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/msgs/wrench.pb.h>

namespace sjtu_drone
{

DroneSimpleControllerPrivate::DroneSimpleControllerPrivate()
  : max_force_(-1.0)
  , motion_small_noise_(0.0)
  , motion_drift_noise_(0.0)
  , motion_drift_noise_time_(1.0)
  , mass(1.0)
  , m_timeAfterCmd(0.0)
  , navi_state(LANDED_MODEL)
  , m_posCtrl(false)
  , m_velMode(false)
  , odom_seq(0)
  , odom_hz(30)
  , pub_odom(false)
  , last_odom_publish_time_(0.0)
{
}

DroneSimpleControllerPrivate::~DroneSimpleControllerPrivate()
{
}

void DroneSimpleControllerPrivate::Reset()
{
  // Reset PID controllers
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();
  controllers_.pos_x.reset();
  controllers_.pos_y.reset();
  controllers_.pos_z.reset();

  // Reset drone state (Phase 4 will handle force/torque reset via ECM)
  pose = gz::math::Pose3d();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  position.Set();

  current_time = 0.0;
  last_time = 0.0;
}

void DroneSimpleControllerPrivate::InitSubscribers(
  std::string cmd_normal_topic_,
  std::string posctrl_topic_,
  std::string imu_topic_,
  std::string takeoff_topic_,
  std::string land_topic_,
  std::string reset_topic_,
  std::string switch_mode_topic_)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  if (!cmd_normal_topic_.empty()) {
    cmd_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_normal_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::CmdCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No cmd_topic defined!");
  }

  if (!posctrl_topic_.empty()) {
    posctrl_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      posctrl_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::PosCtrlCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No position control defined!");
  }

  if (!imu_topic_.empty()) {
    imu_subscriber_ = ros_node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::ImuCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No imu topic defined!");
  }

  if (!takeoff_topic_.empty()) {
    takeoff_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      takeoff_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::TakeoffCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No takeoff topic defined!");
  }

  if (!land_topic_.empty()) {
    land_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      land_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::LandCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No land topic defined!");
  }

  if (!reset_topic_.empty()) {
    reset_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      reset_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::ResetCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No reset topic defined!");
  }

  if (!switch_mode_topic_.empty()) {
    switch_mode_subscriber_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      switch_mode_topic_, qos,
      std::bind(&DroneSimpleControllerPrivate::SwitchModeCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No switch mode topic defined!");
  }
}

void DroneSimpleControllerPrivate::InitPublishers(
  std::string gt_topic_,
  std::string gt_vel_topic_,
  std::string gt_acc_topic_,
  std::string cmd_mode_topic_,
  std::string state_topic_,
  std::string odom_topic_)
{
  if (!gt_topic_.empty()) {
    pub_gt_pose_ = ros_node_->create_publisher<geometry_msgs::msg::Pose>(gt_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth topic defined!");
  }

  if (!gt_vel_topic_.empty()) {
    pub_gt_vec_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>(gt_vel_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth velocity topic defined!");
  }

  if (!gt_acc_topic_.empty()) {
    pub_gt_acc_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>(gt_acc_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No ground truth acceleration topic defined!");
  }

  if (!cmd_mode_topic_.empty()) {
    pub_cmd_mode = ros_node_->create_publisher<std_msgs::msg::String>(cmd_mode_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No command mode topic defined!");
  }

  if (!state_topic_.empty()) {
    pub_state = ros_node_->create_publisher<std_msgs::msg::Int8>(state_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No state topic defined!");
  }

  if (!odom_topic_.empty()) {
    pub_odom_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 1024);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "No odom topic defined!");
  }
}

void DroneSimpleControllerPrivate::LoadControllerSettings(sdf::ElementPtr _sdf)
{
  controllers_.roll.Load(_sdf, "rollpitch");
  controllers_.pitch.Load(_sdf, "rollpitch");
  controllers_.yaw.Load(_sdf, "yaw");
  controllers_.velocity_x.Load(_sdf, "velocityXY");
  controllers_.velocity_y.Load(_sdf, "velocityXY");
  controllers_.velocity_z.Load(_sdf, "velocityZ");

  controllers_.pos_x.Load(_sdf, "positionXY");
  controllers_.pos_y.Load(_sdf, "positionXY");
  controllers_.pos_z.Load(_sdf, "positionZ");

  RCLCPP_INFO_STREAM(
    ros_node_->get_logger(), "Using the PID parameters: \n" <<
      "\tRoll Pitch:\n" << "\t\tkP: " << controllers_.roll.gain_p <<
      ", kI: " << controllers_.roll.gain_i <<
      ", kD: " << controllers_.roll.gain_d <<
      ", Limit: " << controllers_.roll.limit <<
      ", Time Constant: " << controllers_.roll.time_constant << "\n" <<
      "\tYaw:\n" << "\t\tkP: " << controllers_.yaw.gain_p <<
      ", kI: " << controllers_.yaw.gain_i <<
      ", kD: " << controllers_.yaw.gain_d <<
      ", Limit: " << controllers_.yaw.limit <<
      ", Time Constant: " << controllers_.yaw.time_constant << "\n" <<
      "\tVelocity XY:\n" << "\t\tkP: " << controllers_.velocity_x.gain_p <<
      ", kI: " << controllers_.velocity_x.gain_i <<
      ", kD: " << controllers_.velocity_x.gain_d <<
      ", Limit: " << controllers_.velocity_x.limit <<
      ", Time Constant: " << controllers_.velocity_x.time_constant << "\n" <<
      "\tVelocity Z:\n" << "\t\tkP: " << controllers_.velocity_z.gain_p <<
      ", kI: " << controllers_.velocity_z.gain_i <<
      ", kD: " << controllers_.velocity_z.gain_d <<
      ", Limit: " << controllers_.velocity_z.limit <<
      ", Time Constant: " << controllers_.velocity_z.time_constant << "\n" <<
      "\tPosition XY:\n" << "\t\tkP: " << controllers_.pos_x.gain_p <<
      ", kI: " << controllers_.pos_x.gain_i <<
      ", kD: " << controllers_.pos_x.gain_d <<
      ", Limit: " << controllers_.pos_x.limit <<
      ", Time Constant: " << controllers_.pos_x.time_constant << "\n" <<
      "\tPosition Z:\n" << "\t\tkP: " << controllers_.pos_z.gain_p <<
      ", kI: " << controllers_.pos_z.gain_i <<
      ", kD: " << controllers_.pos_z.gain_d <<
      ", Limit: " << controllers_.pos_z.limit <<
      ", Time Constant: " << controllers_.pos_z.time_constant);
}

// Callbacks

void DroneSimpleControllerPrivate::CmdCallback(
  const geometry_msgs::msg::Twist::SharedPtr cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_vel = *cmd;

  static double last_cmd_time = current_time;
  static double time_counter_for_drift_noise = 0;
  static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};

  double dt = current_time - last_cmd_time;
  last_cmd_time = current_time;

  // Generate noise
  if (time_counter_for_drift_noise > motion_drift_noise_time_) {
    drift_noise[0] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    drift_noise[1] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    drift_noise[2] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    drift_noise[3] = 2 * motion_drift_noise_ * (drand48() - 0.5);
    time_counter_for_drift_noise = 0.0;
  }
  time_counter_for_drift_noise += dt;

  cmd_vel.angular.x += drift_noise[0] + 2 * motion_small_noise_ * (drand48() - 0.5);
  cmd_vel.angular.y += drift_noise[1] + 2 * motion_small_noise_ * (drand48() - 0.5);
  cmd_vel.angular.z += drift_noise[3] + 2 * motion_small_noise_ * (drand48() - 0.5);
  cmd_vel.linear.z += drift_noise[2] + 2 * motion_small_noise_ * (drand48() - 0.5);
}

void DroneSimpleControllerPrivate::PosCtrlCallback(
  const std_msgs::msg::Bool::SharedPtr cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  m_posCtrl = cmd->data;
}

void DroneSimpleControllerPrivate::ImuCallback(
  const sensor_msgs::msg::Imu::SharedPtr imu)
{
  pose.Rot().Set(
    imu->orientation.w, imu->orientation.x,
    imu->orientation.y, imu->orientation.z);
  euler = pose.Rot().Euler();
  angular_velocity = pose.Rot().RotateVector(
    gz::math::Vector3d(
      imu->angular_velocity.x,
      imu->angular_velocity.y,
      imu->angular_velocity.z));
}

void DroneSimpleControllerPrivate::TakeoffCallback(
  const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (navi_state == LANDED_MODEL) {
    navi_state = TAKINGOFF_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(ros_node_->get_logger(), "Quadrotor takes off!!");
  }
}

void DroneSimpleControllerPrivate::LandCallback(
  const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (navi_state == FLYING_MODEL || navi_state == TAKINGOFF_MODEL) {
    navi_state = LANDING_MODEL;
    m_timeAfterCmd = 0;
    RCLCPP_INFO(ros_node_->get_logger(), "Quadrotor lands!!");
  }
}

void DroneSimpleControllerPrivate::ResetCallback(
  const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Reset quadrotor!!");
  std::lock_guard<std::mutex> lock(mutex_);
  reset_requested = true;
}

void DroneSimpleControllerPrivate::SwitchModeCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  m_velMode = msg->data;

  std_msgs::msg::String mode;
  if (m_velMode) {
    mode.data = "velocity";
  } else {
    mode.data = "position";
  }
  pub_cmd_mode->publish(mode);
}

void DroneSimpleControllerPrivate::PublishOdom(
  const gz::math::Pose3d &pose,
  const gz::math::Vector3d &velocity,
  const gz::math::Vector3d &/*acceleration*/)
{
  nav_msgs::msg::Odometry odom;

  // Convert sim time to ROS time
  int32_t sec = static_cast<int32_t>(current_time);
  uint32_t nsec = static_cast<uint32_t>((current_time - sec) * 1e9);
  odom.header.stamp.sec = sec;
  odom.header.stamp.nanosec = nsec;

  std::string ns = ros_node_->get_namespace();
  if (ns == "/") ns = "";
  odom.header.frame_id = ns + "/odom";
  odom.child_frame_id = ns + "/base_footprint";

  // Position and orientation
  odom.pose.pose.position.x = pose.Pos().X();
  odom.pose.pose.position.y = pose.Pos().Y();
  odom.pose.pose.position.z = pose.Pos().Z();
  odom.pose.pose.orientation.w = pose.Rot().W();
  odom.pose.pose.orientation.x = pose.Rot().X();
  odom.pose.pose.orientation.y = pose.Rot().Y();
  odom.pose.pose.orientation.z = pose.Rot().Z();

  // Velocity in child frame
  auto pose_rot_inv = pose.Rot().Inverse();
  auto linear_child = pose_rot_inv.RotateVector(velocity);
  auto angular_child = pose_rot_inv.RotateVector(angular_velocity);
  odom.twist.twist.linear.x = linear_child.X();
  odom.twist.twist.linear.y = linear_child.Y();
  odom.twist.twist.linear.z = linear_child.Z();
  odom.twist.twist.angular.x = angular_child.X();
  odom.twist.twist.angular.y = angular_child.Y();
  odom.twist.twist.angular.z = angular_child.Z();

  pub_odom_->publish(odom);

  // TF broadcast
  geometry_msgs::msg::TransformStamped tf;
  tf.header = odom.header;
  tf.child_frame_id = odom.child_frame_id;
  tf.transform.translation.x = pose.Pos().X();
  tf.transform.translation.y = pose.Pos().Y();
  tf.transform.translation.z = pose.Pos().Z();
  tf.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf);
}

void DroneSimpleControllerPrivate::UpdateState(double dt)
{
  if (navi_state == TAKINGOFF_MODEL) {
    m_timeAfterCmd += dt;
    if (m_timeAfterCmd > 0.5) {
      navi_state = FLYING_MODEL;
      std::cout << "Entering flying model!" << std::endl;
    }
  } else if (navi_state == LANDING_MODEL) {
    m_timeAfterCmd += dt;
    if (m_timeAfterCmd > 1.0) {
      navi_state = LANDED_MODEL;
      std::cout << "Landed!" << std::endl;
    }
  } else {
    m_timeAfterCmd = 0;
  }

  // Publish current state
  std_msgs::msg::Int8 state_msg;
  state_msg.data = navi_state;
  pub_state->publish(state_msg);
}

void DroneSimpleControllerPrivate::UpdateDynamics(
  double dt,
  gz::sim::EntityComponentManager &_ecm)
{
  gz::math::Vector3d force, torque;

  // --- Read pose from ECM ---
  gz::sim::Link link(link_entity);

  auto poseOpt = link.WorldPose(_ecm);
  if (!poseOpt) {
    _ecm.CreateComponent(link_entity, gz::sim::components::WorldPose());
    return;
  }
  pose = poseOpt.value();

  // --- Read angular velocity from ECM ---
  auto angVelOpt = link.WorldAngularVelocity(_ecm);
  if (!angVelOpt) {
    _ecm.CreateComponent(link_entity, gz::sim::components::WorldAngularVelocity());
    return;
  }
  angular_velocity = angVelOpt.value();

  // --- Read linear velocity from ECM ---
  auto linVelOpt = link.WorldLinearVelocity(_ecm);
  if (!linVelOpt) {
    _ecm.CreateComponent(link_entity, gz::sim::components::WorldLinearVelocity());
    return;
  }
  auto new_velocity = linVelOpt.value();

  euler = pose.Rot().Euler();
  acceleration = (new_velocity - velocity) / dt;
  velocity = new_velocity;

  // Publish ground truth pose
  geometry_msgs::msg::Pose gt_pose;
  gt_pose.position.x = pose.Pos().X();
  gt_pose.position.y = pose.Pos().Y();
  gt_pose.position.z = pose.Pos().Z();
  gt_pose.orientation.w = pose.Rot().W();
  gt_pose.orientation.x = pose.Rot().X();
  gt_pose.orientation.y = pose.Rot().Y();
  gt_pose.orientation.z = pose.Rot().Z();
  pub_gt_pose_->publish(gt_pose);

  // Convert to body frame for ground truth velocity/acceleration
  gz::math::Vector3d body_vel = pose.Rot().RotateVector(velocity);
  gz::math::Vector3d body_acc = pose.Rot().RotateVector(acceleration);

  // Publish ground truth velocity
  geometry_msgs::msg::Twist tw;
  tw.linear.x = body_vel.X();
  tw.linear.y = body_vel.Y();
  tw.linear.z = body_vel.Z();
  pub_gt_vec_->publish(tw);

  // Publish ground truth acceleration
  tw.linear.x = body_acc.X();
  tw.linear.y = body_acc.Y();
  tw.linear.z = body_acc.Z();
  pub_gt_acc_->publish(tw);

  gz::math::Vector3d poschange = pose.Pos() - position;
  position = pose.Pos();

  // Get gravity
  auto gravityComp = _ecm.Component<gz::sim::components::Gravity>(world_entity);
  gz::math::Vector3d world_gravity = gravityComp ?
    gravityComp->Data() : gz::math::Vector3d(0, 0, -9.81);

  gz::math::Vector3d gravity_body = pose.Rot().RotateVector(world_gravity);
  double gravity = gravity_body.Length();
  double load_factor = gravity * gravity / world_gravity.Dot(gravity_body);

  // Rotate vectors to coordinate frames relevant for control
  gz::math::Quaterniond heading_quaternion(
    cos(euler[2] / 2), 0.0, 0.0, sin(euler[2] / 2));
  gz::math::Vector3d velocity_xy =
    heading_quaternion.RotateVectorReverse(velocity);
  gz::math::Vector3d acceleration_xy =
    heading_quaternion.RotateVectorReverse(acceleration);
  gz::math::Vector3d angular_velocity_body =
    pose.Rot().RotateVectorReverse(angular_velocity);

  // Update controllers
  force.Set(0.0, 0.0, 0.0);
  torque.Set(0.0, 0.0, 0.0);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (m_posCtrl) {
      // Position control
      if (navi_state == FLYING_MODEL) {
        double vx = controllers_.pos_x.update(
          cmd_vel.linear.x, position[0], poschange[0], dt);
        double vy = controllers_.pos_y.update(
          cmd_vel.linear.y, position[1], poschange[1], dt);
        double vz = controllers_.pos_z.update(
          cmd_vel.linear.z, position[2], poschange[2], dt);

        gz::math::Vector3d vb = heading_quaternion.RotateVectorReverse(
          gz::math::Vector3d(vx, vy, vz));

        double pitch_command = controllers_.velocity_x.update(
          vb[0], velocity_xy[0], acceleration_xy[0], dt) / gravity;
        double roll_command = -controllers_.velocity_y.update(
          vb[1], velocity_xy[1], acceleration_xy[1], dt) / gravity;
        torque[0] = inertia[0] * controllers_.roll.update(
          roll_command, euler[0], angular_velocity_body[0], dt);
        torque[1] = inertia[1] * controllers_.pitch.update(
          pitch_command, euler[1], angular_velocity_body[1], dt);
        force[2] = mass *
          (controllers_.velocity_z.update(
            vz, velocity[2], acceleration[2], dt) + load_factor * gravity);
      }
    } else {
      // Normal control
      if (navi_state == FLYING_MODEL) {
        // Hovering
        double pitch_command = controllers_.velocity_x.update(
          cmd_vel.linear.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
        double roll_command = -controllers_.velocity_y.update(
          cmd_vel.linear.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
        torque[0] = inertia[0] * controllers_.roll.update(
          roll_command, euler[0], angular_velocity_body[0], dt);
        torque[1] = inertia[1] * controllers_.pitch.update(
          pitch_command, euler[1], angular_velocity_body[1], dt);
      } else {
        // Control by velocity or tilting
        if (m_velMode) {
          double pitch_command = controllers_.velocity_x.update(
            cmd_vel.angular.x, velocity_xy[0], acceleration_xy[0], dt) / gravity;
          double roll_command = -controllers_.velocity_y.update(
            cmd_vel.angular.y, velocity_xy[1], acceleration_xy[1], dt) / gravity;
          torque[0] = inertia[0] * controllers_.roll.update(
            roll_command, euler[0], angular_velocity_body[0], dt);
          torque[1] = inertia[1] * controllers_.pitch.update(
            pitch_command, euler[1], angular_velocity_body[1], dt);
        } else {
          // Control by tilting
          torque[0] = inertia[0] * controllers_.roll.update(
            cmd_vel.angular.x, euler[0], angular_velocity_body[0], dt);
          torque[1] = inertia[1] * controllers_.pitch.update(
            cmd_vel.angular.y, euler[1], angular_velocity_body[1], dt);
        }
      }
      torque[2] = inertia[2] * controllers_.yaw.update(
        cmd_vel.angular.z, angular_velocity[2], 0, dt);
      force[2] = mass *
        (controllers_.velocity_z.update(
          cmd_vel.linear.z, velocity[2], acceleration[2], dt) + load_factor * gravity);
    }
  }  // mutex unlocked

  if (max_force_ > 0.0 && force[2] > max_force_) { force[2] = max_force_; }
  if (force[2] < 0.0) { force[2] = 0.0; }

  // Apply force multiplier based on state
  gz::math::Vector3d applied_force;
  gz::math::Vector3d applied_torque;

  if (navi_state == LANDED_MODEL) {
    applied_force.Set(0, 0, 0);
    applied_torque.Set(0, 0, 0);
  } else if (navi_state == FLYING_MODEL) {
    applied_force = force;
    applied_torque = torque;
  } else if (navi_state == TAKINGOFF_MODEL) {
    applied_force = force * 1.5;
    applied_torque = torque * 1.5;
  } else if (navi_state == LANDING_MODEL) {
    applied_force = force * 0.8;
    applied_torque = torque * 0.8;
  }

  // Convert relative (body) force/torque to world frame
  gz::math::Vector3d world_force = pose.Rot().RotateVector(applied_force);
  gz::math::Vector3d world_torque = pose.Rot().RotateVector(applied_torque);

  // Apply via ECM ExternalWorldWrenchCmd component
  gz::msgs::Wrench wrench_msg;
  wrench_msg.mutable_force()->set_x(world_force.X());
  wrench_msg.mutable_force()->set_y(world_force.Y());
  wrench_msg.mutable_force()->set_z(world_force.Z());
  wrench_msg.mutable_torque()->set_x(world_torque.X());
  wrench_msg.mutable_torque()->set_y(world_torque.Y());
  wrench_msg.mutable_torque()->set_z(world_torque.Z());

  auto wrenchComp =
    _ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(link_entity);
  if (wrenchComp) {
    *wrenchComp = gz::sim::components::ExternalWorldWrenchCmd(wrench_msg);
  } else {
    _ecm.CreateComponent(link_entity,
      gz::sim::components::ExternalWorldWrenchCmd(wrench_msg));
  }

  // Publish odometry
  if (pub_odom) {
    if (current_time - last_odom_publish_time_ >= 1.0 / odom_hz) {
      PublishOdom(pose, velocity, acceleration);
      last_odom_publish_time_ = current_time;
      odom_seq++;
    }
  }
}

}  // namespace sjtu_drone
