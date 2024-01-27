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

#ifndef GAZEBO_PLUGINS_DRONE_SIMPLE_H
#define GAZEBO_PLUGINS_DRONE_SIMPLE_H

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "sdf/sdf.hh"

#include "sjtu_drone_description/plugin_drone_private.h"


using namespace std::placeholders;

namespace gazebo_plugins
{

class DroneSimpleController : public gazebo::ModelPlugin
{
public:
  DroneSimpleController(void);
  virtual ~DroneSimpleController(void);

protected:
  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update(const gazebo::common::UpdateInfo & _info);
  virtual void Reset(void);

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

#endif // GAZEBO_PLUGINS_DRONE_SIMPLE_H
