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

#ifndef SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_H_
#define SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_H_

#include <memory>
#include <string>

#include <gz/sim/System.hh>

namespace sjtu_drone
{

// Forward declare
class DroneSimpleControllerPrivate;

class DroneSimpleController
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  DroneSimpleController();
  ~DroneSimpleController() override;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  std::unique_ptr<DroneSimpleControllerPrivate> impl_;
};

}  // namespace sjtu_drone

#endif  // SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_H_
