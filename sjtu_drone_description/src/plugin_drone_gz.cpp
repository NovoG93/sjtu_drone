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

#include "sjtu_drone_description/plugin_drone.h"
#include "sjtu_drone_description/plugin_drone_private.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/wrench.pb.h>

#include <chrono>

namespace sjtu_drone
{

DroneSimpleController::DroneSimpleController()
  : impl_(std::make_unique<DroneSimpleControllerPrivate>())
{
}

DroneSimpleController::~DroneSimpleController()
{
  if (impl_->executor_) {
    impl_->executor_->cancel();
  }
  if (impl_->executor_thread_.joinable()) {
    impl_->executor_thread_.join();
  }
}

void DroneSimpleController::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/)
{
  // Initialize ROS if not already done
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Create ROS node with namespace from SDF
  std::string node_name = "drone_simple_controller";
  std::string ns = "";
  auto sdf_tmp = _sdf->Clone();
  if (sdf_tmp->HasElement("robotNamespace")) {
    ns = sdf_tmp->GetElement("robotNamespace")->Get<std::string>();
  }
  impl_->ros_node_ = rclcpp::Node::make_shared(node_name, ns);
  impl_->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

  // Store model entity
  gz::sim::Model model(_entity);
  impl_->model_entity = _entity;

  // Get world entity
  impl_->world_entity = gz::sim::worldEntity(_ecm);

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin is loading!");

  // Clone SDF for mutable access
  auto sdf = _sdf->Clone();

  // Get link
  if (!sdf->HasElement("bodyName")) {
    auto canonical_link = model.CanonicalLink(_ecm);
    if (canonical_link != gz::sim::kNullEntity) {
      impl_->link_entity = canonical_link;
      auto nameComp = _ecm.Component<gz::sim::components::Name>(canonical_link);
      if (nameComp) {
        impl_->link_name = nameComp->Data();
      }
    } else {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Model has no canonical link!");
      return;
    }
  } else {
    impl_->link_name = sdf->GetElement("bodyName")->Get<std::string>();
    auto link_entity = model.LinkByName(_ecm, impl_->link_name);
    if (link_entity == gz::sim::kNullEntity) {
      RCLCPP_FATAL(
        impl_->ros_node_->get_logger(),
        "bodyName: %s does not exist", impl_->link_name.c_str());
      return;
    }
    impl_->link_entity = link_entity;
  }

  // Read pub_odom setting
  if (!sdf->HasElement("pub_odom")) {
    impl_->pub_odom = false;
  } else {
    impl_->pub_odom = sdf->GetElement("pub_odom")->Get<bool>();
    if (!sdf->HasElement("odom_hz")) {
      impl_->odom_hz = 30;
    } else {
      impl_->odom_hz = sdf->GetElement("odom_hz")->Get<int>();
    }
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "pub_odom: %d", impl_->pub_odom);

  // Read max force
  if (!sdf->HasElement("maxForce")) {
    impl_->max_force_ = -1;
  } else {
    impl_->max_force_ = sdf->GetElement("maxForce")->Get<double>();
  }

  // Read noise parameters
  if (!sdf->HasElement("motionSmallNoise")) {
    impl_->motion_small_noise_ = 0;
  } else {
    impl_->motion_small_noise_ = sdf->GetElement("motionSmallNoise")->Get<double>();
  }

  if (!sdf->HasElement("motionDriftNoise")) {
    impl_->motion_drift_noise_ = 0;
  } else {
    impl_->motion_drift_noise_ = sdf->GetElement("motionDriftNoise")->Get<double>();
  }

  if (!sdf->HasElement("motionDriftNoiseTime")) {
    impl_->motion_drift_noise_time_ = 1.0;
  } else {
    impl_->motion_drift_noise_time_ = sdf->GetElement("motionDriftNoiseTime")->Get<double>();
  }

  RCLCPP_INFO_STREAM(
    impl_->ros_node_->get_logger(), "Using following parameters: \n" <<
      "\t\tlink_name: " << impl_->link_name.c_str() << ",\n" <<
      "\t\tmax_force: " << impl_->max_force_ << ",\n" <<
      "\t\tmotion_small_noise: " << impl_->motion_small_noise_ << ",\n" <<
      "\t\tmotion_drift_noise: " << impl_->motion_drift_noise_ << ",\n" <<
      "\t\tmotion_drift_noise_time: " << impl_->motion_drift_noise_time_);

  // Get inertia and mass from ECM
  auto inertialComp = _ecm.Component<gz::sim::components::Inertial>(impl_->link_entity);
  if (inertialComp) {
    auto inertial = inertialComp->Data();
    impl_->mass = inertial.MassMatrix().Mass();
    impl_->inertia = gz::math::Vector3d(
      inertial.MassMatrix().Ixx(),
      inertial.MassMatrix().Iyy(),
      inertial.MassMatrix().Izz());
  } else {
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "No Inertial component found on link");
    impl_->mass = 1.0;
    impl_->inertia = gz::math::Vector3d(1.0, 1.0, 1.0);
  }

  // Enable pose/velocity components so they're available in PreUpdate
  _ecm.CreateComponent(impl_->link_entity, gz::sim::components::WorldPose());
  _ecm.CreateComponent(impl_->link_entity, gz::sim::components::WorldLinearVelocity());
  _ecm.CreateComponent(impl_->link_entity, gz::sim::components::WorldAngularVelocity());

  // Initialize ROS subscribers and publishers
  impl_->InitSubscribers();
  impl_->InitPublishers();

  // Load PID controller settings from SDF
  impl_->LoadControllerSettings(sdf);

  // Reset state
  impl_->Reset();

  // Start ROS executor in a background thread
  impl_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  impl_->executor_->add_node(impl_->ros_node_);
  impl_->executor_thread_ = std::thread([this]() {
    impl_->executor_->spin();
  });

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin finished loading!");
}

void DroneSimpleController::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) return;

  double sim_time = std::chrono::duration<double>(_info.simTime).count();
  double dt = sim_time - impl_->last_time;
  if (dt <= 0.0) return;

  impl_->current_time = sim_time;

  // Handle reset request from ROS callback thread
  if (impl_->reset_requested) {
    impl_->Reset();
    // Zero out wrench
    gz::msgs::Wrench zero;
    zero.mutable_force()->set_x(0);
    zero.mutable_force()->set_y(0);
    zero.mutable_force()->set_z(0);
    zero.mutable_torque()->set_x(0);
    zero.mutable_torque()->set_y(0);
    zero.mutable_torque()->set_z(0);
    auto wrenchComp =
      _ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(impl_->link_entity);
    if (wrenchComp) {
      *wrenchComp = gz::sim::components::ExternalWorldWrenchCmd(zero);
    }
    impl_->reset_requested = false;
    return;
  }

  impl_->UpdateState(dt);
  impl_->UpdateDynamics(dt, _ecm);

  impl_->last_time = impl_->current_time;
}

}  // namespace sjtu_drone

GZ_ADD_PLUGIN(sjtu_drone::DroneSimpleController,
              gz::sim::System,
              sjtu_drone::DroneSimpleController::ISystemConfigure,
              sjtu_drone::DroneSimpleController::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(sjtu_drone::DroneSimpleController,
                    "sjtu_drone::DroneSimpleController")
