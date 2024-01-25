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

#ifndef PLUGIN_DRONE_H
#define PLUGIN_DRONE_H

#include "gazebo_ros/node.hpp"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>

#include "pid_controller.h"

#define LANDED_MODEL 0
#define FLYING_MODEL 1
#define TAKINGOFF_MODEL 2
#define LANDING_MODEL 3

using namespace std::placeholders;

namespace gazebo_plugins
{

// Forward declaration of pimpl idiom
class DroneSimpleControllerPrivate;

class DroneSimpleController : public gazebo::ModelPlugin
{
public:
  DroneSimpleController();
  virtual ~DroneSimpleController();

protected:
  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  std::unique_ptr<DroneSimpleControllerPrivate> impl_;  // Forward declaration of pimpl idiom

  gazebo::event::ConnectionPtr updateConnection; // Pointer to the update event connection

  /// \brief The parent World
  gazebo::physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  gazebo::physics::LinkPtr link;
  std::string link_name_;

  /// \brief save last_time
  gazebo::common::Time last_time;
};

}

#endif // PLUGIN_DRONE_HPP