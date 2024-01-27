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


#include <cmath>
#include <stdlib.h>
#include <iostream>


namespace gazebo_plugins
{

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
  impl_->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
  impl_->model = _model;

  if (!rclcpp::ok()) {
    RCLCPP_FATAL(
      impl_->ros_node_->get_logger(),
      "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
    exit(1);
  }

  world = _model->GetWorld();
  impl_->world = _model->GetWorld();
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin is loading!");

  if (!_sdf->HasElement("bodyName")) {
    impl_->link = _model->GetLink();
    link_name_ = impl_->link->GetName();
  } else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    // auto entity = world->EntityByName(link_name_);
    // impl_->link = boost::dynamic_pointer_cast<gazebo::physics::Link>(entity);
    impl_->link =
      boost::dynamic_pointer_cast<gazebo::physics::Link>(world->EntityByName(link_name_));
  }

  if (!impl_->link) {
    RCLCPP_FATAL(
      impl_->ros_node_->get_logger(), "gazebo_ros_baro plugin error: bodyName: %s does not exist\n",
      link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("pub_odom")) {
    impl_->pub_odom = false;
  } else {
    impl_->pub_odom = _sdf->GetElement("pub_odom")->Get<bool>();
    if(!_sdf->HasElement("odom_hz")) {
      impl_->odom_hz = 30;
    } else {
      impl_->odom_hz = _sdf->GetElement("odom_hz")->Get<double>();
    }
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "pub_odom: %d", impl_->pub_odom);

  if (!_sdf->HasElement("maxForce")) {
    impl_->max_force_ = -1;
  } else {
    impl_->max_force_ = _sdf->GetElement("maxForce")->Get<double>();
  }

  if (!_sdf->HasElement("motionSmallNoise")) {
    impl_->motion_small_noise_ = 0;
  } else {
    impl_->motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();
  }

  if (!_sdf->HasElement("motionDriftNoise")) {
    impl_->motion_drift_noise_ = 0;
  } else {
    impl_->motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();
  }

  if (!_sdf->HasElement("motionDriftNoiseTime")) {
    impl_->motion_drift_noise_time_ = 1.0;
  } else {
    impl_->motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();
  }


  RCLCPP_INFO_STREAM(
    impl_->ros_node_->get_logger(), "Using following parameters: \n" <<
      "\t\tlink_name: " << link_name_.c_str() << ",\n" <<
      "\t\tmax_force: " << impl_->max_force_ << ",\n" <<
      "\t\tmotion_small_noise: " << impl_->motion_small_noise_ << ",\n" <<
      "\t\tmotion_drift_noise: " << impl_->motion_drift_noise_ << ",\n" <<
      "\t\tmotion_drift_noise_time: " << impl_->motion_drift_noise_time_
  );

  // get inertia and mass of quadrotor body
  impl_->inertia = impl_->link->GetInertial()->PrincipalMoments();
  impl_->mass = impl_->link->GetInertial()->Mass();


  impl_->InitSubscribers();
  impl_->InitPublishers();

  impl_->LoadControllerSettings(_model, _sdf);

  Reset();

  updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&DroneSimpleController::Update, this,  std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "The drone plugin finished loading!");
}

/**
* @brief Update method called by the Gazebo simulator every simulation iteration.
*/
void DroneSimpleController::Update(const gazebo::common::UpdateInfo & _info)
{
  impl_->current_time = _info.simTime;
  const double dt = (impl_->current_time - impl_->last_time).Double();
  if (dt == 0.0) {return;}

  impl_->UpdateState(dt);
  impl_->UpdateDynamics(dt);

  // save last time stamp
  impl_->last_time = impl_->current_time;
}

void DroneSimpleController::Reset()
{
  impl_->Reset();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo_plugins
