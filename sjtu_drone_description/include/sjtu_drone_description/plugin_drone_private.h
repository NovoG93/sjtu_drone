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

#ifndef SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_PRIVATE_H_
#define SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_PRIVATE_H_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sjtu_drone_description/pid_controller.h"

#define LANDED_MODEL 0
#define FLYING_MODEL 1
#define TAKINGOFF_MODEL 2
#define LANDING_MODEL 3

namespace sjtu_drone
{

class DroneSimpleControllerPrivate
{
public:
  DroneSimpleControllerPrivate();
  ~DroneSimpleControllerPrivate();

  void Reset();
  void InitSubscribers(
    std::string cmd_normal_topic_ = "cmd_vel",
    std::string posctrl_topic_ = "posctrl",
    std::string imu_topic_ = "imu",
    std::string takeoff_topic_ = "takeoff",
    std::string land_topic_ = "land",
    std::string reset_topic_ = "reset",
    std::string switch_mode_topic_ = "dronevel_mode");
  void InitPublishers(
    std::string gt_topic_ = "gt_pose",
    std::string gt_vel_topic_ = "gt_vel",
    std::string gt_acc_topic_ = "gt_acc",
    std::string cmd_mode_topic_ = "cmd_mode",
    std::string state_topic_ = "state",
    std::string odom_topic_ = "odom");
  void LoadControllerSettings(sdf::ElementPtr _sdf);

  void UpdateState(double dt);
  void UpdateDynamics(double dt, gz::sim::EntityComponentManager &_ecm);

  // gz-sim entity handles
  gz::sim::Entity model_entity{gz::sim::kNullEntity};
  gz::sim::Entity link_entity{gz::sim::kNullEntity};
  gz::sim::Entity world_entity{gz::sim::kNullEntity};
  std::string link_name;

  // Simulation configuration
  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;

  // Inertia and mass
  gz::math::Vector3d inertia;
  double mass;
  double m_timeAfterCmd;
  int navi_state;
  bool m_posCtrl;
  bool m_velMode;
  int odom_seq;
  int odom_hz;
  bool pub_odom;

  // Timing
  double current_time{0.0};
  double last_time{0.0};

  // Thread safety
  std::mutex mutex_;
  bool reset_requested{false};

  // rclcpp
  rclcpp::Node::SharedPtr ros_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Executor for spinning the ROS node
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

private:
  // Callbacks
  void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd);
  void PosCtrlCallback(const std_msgs::msg::Bool::SharedPtr cmd);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);
  void TakeoffCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void LandCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void ResetCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void SwitchModeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  double last_odom_publish_time_;
public:
  void PublishOdom(
    const gz::math::Pose3d &pose,
    const gz::math::Vector3d &velocity,
    const gz::math::Vector3d &acceleration);
private:

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

  // ROS messages
  geometry_msgs::msg::Twist cmd_vel;
  nav_msgs::msg::Odometry odom;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr posctrl_subscriber_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr takeoff_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr land_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_mode_subscriber_{nullptr};

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_gt_pose_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_vec_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_gt_acc_{nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_mode{nullptr};
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_state{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_{nullptr};

  // PID Controllers
  Controllers controllers_;

  // Drone state
  gz::math::Pose3d pose;
  gz::math::Vector3d euler;
  gz::math::Vector3d velocity, acceleration, angular_velocity, position;
};

}  // namespace sjtu_drone

#endif  // SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_PRIVATE_H_
