//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
// This code is based on the original gazebo_ros_imu plugin by Sachin Chitta and John Hsu:
/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
//=================================================================================================

#include "plugin_ros_imu.h"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{

// #define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
  #include <geometry_msgs/PoseStamped.h>
  static ros::Publisher debugPublisher;
#endif // DEBUG_OUTPUT

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  node_handle_->shutdown();
#ifdef USE_CBQ
  callback_queue_thread_.join();
#endif
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO("GazeboRosIMU::Load");
  // Get the world name.
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    robotNamespace.clear();
  else
    robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    linkName = link->GetName();
  }
  else {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    link = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(linkName));
  }

  // assert that the body by linkName exists
  if (!link)
  {
    ROS_FATAL("GazeboRosIMU plugin error: bodyName: %s does not exist\n", linkName.c_str());
    return;
  }

  double update_rate = 0.0;
  if (_sdf->HasElement("updateRate")) update_rate = _sdf->GetElement("updateRate")->Get<double>();
  update_period = update_rate > 0.0 ? 1.0/update_rate : 100.0;

  if (!_sdf->HasElement("frameId"))
    frameId = linkName;
  else
    frameId = _sdf->GetElement("frameId")->Get<std::string>();

  if (!_sdf->HasElement("topicName"))
    topicName = "imu";
  else
    topicName = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("serviceName"))
    serviceName = topicName + "/calibrate";
  else
    serviceName = _sdf->GetElement("serviceName")->Get<std::string>();

  accelModel.Load(_sdf, "accel");
  rateModel.Load(_sdf, "rate");
  headingModel.Load(_sdf, "heading");

//  // also use old configuration variables from gazebo_ros_imu
//  if (_sdf->HasElement("gaussianNoise")) {
//    double gaussianNoise = _sdf->GetElement("gaussianNoise")->Get<double>();
//    if (gaussianNoise != 0.0) {
//        //test
//        std::cout << "IMU noise:" << gaussianNoise << std::endl;
//        accelModel.gaussian_noise = gaussianNoise;
//        rateModel.gaussian_noise  = gaussianNoise;
//    }
//  }

  if (_sdf->HasElement("rpyOffset")) {
    sdf::Vector3 rpyOffset = _sdf->GetElement("rpyOffset")->Get<sdf::Vector3>();
    if (accelModel.offset.y == 0.0 && rpyOffset.x != 0.0) accelModel.offset.y = -rpyOffset.x * 9.8065;
    if (accelModel.offset.x == 0.0 && rpyOffset.y != 0.0) accelModel.offset.x =  rpyOffset.y * 9.8065;
    if (headingModel.offset == 0.0 && rpyOffset.z != 0.0) headingModel.offset =  rpyOffset.z;
  }

  //test
  std::cout << "accel noise:" << accelModel.gaussian_noise.x << " " 
            << accelModel.gaussian_noise.y << " " << accelModel.gaussian_noise.z << std::endl;
  
  // fill in constant covariance matrix
  imuMsg.angular_velocity_covariance[0] = rateModel.gaussian_noise.x*rateModel.gaussian_noise.x;
  imuMsg.angular_velocity_covariance[4] = rateModel.gaussian_noise.y*rateModel.gaussian_noise.y;
  imuMsg.angular_velocity_covariance[8] = rateModel.gaussian_noise.z*rateModel.gaussian_noise.z;
  imuMsg.linear_acceleration_covariance[0] = accelModel.gaussian_noise.x*accelModel.gaussian_noise.x;
  imuMsg.linear_acceleration_covariance[4] = accelModel.gaussian_noise.y*accelModel.gaussian_noise.y;
  imuMsg.linear_acceleration_covariance[8] = accelModel.gaussian_noise.z*accelModel.gaussian_noise.z;

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(robotNamespace);

  // if topic name specified as empty, do not publish (then what is this plugin good for?)
  if (!topicName.empty())
    pub_ = node_handle_->advertise<sensor_msgs::Imu>(topicName, 1);

#ifdef DEBUG_OUTPUT
  debugPublisher = rosnode_->advertise<geometry_msgs::PoseStamped>(topicName + "/pose", 10);
#endif // DEBUG_OUTPUT

  // advertise services for calibration and bias setting
  if (!serviceName.empty())
    srv_ = node_handle_->advertiseService(serviceName, &GazeboRosIMU::ServiceCalibrate, this);

  accelBiasService = node_handle_->advertiseService(topicName + "/set_accel_bias", &GazeboRosIMU::SetAccelBiasCallback, this);
  rateBiasService  = node_handle_->advertiseService(topicName + "/set_rate_bias", &GazeboRosIMU::SetRateBiasCallback, this);

#ifdef USE_CBQ
  // start custom queue for imu
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::CallbackQueueThread,this ) );
#endif

  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosIMU::Update, this));
}

void GazeboRosIMU::Reset()
{
  last_time = world->SimTime();
  orientation = ignition::math::Quaternion();
  velocity = 0.0;
  accel = 0.0;

  accelModel.reset();
  rateModel.reset();
  headingModel.reset();
}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCalibrate(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  rateModel.reset();
  return true;
}

bool GazeboRosIMU::SetAccelBiasCallback(sjtu_drone::SetBias::Request &req, sjtu_drone::SetBias::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  accelModel.reset(ignition::math::Vector3(req.bias.x, req.bias.y, req.bias.z));
  return true;
}

bool GazeboRosIMU::SetRateBiasCallback(sjtu_drone::SetBias::Request &req, sjtu_drone::SetBias::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  rateModel.reset(ignition::math::Vector3(req.bias.x, req.bias.y, req.bias.z));
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::Update()
{
  // Get Time Difference dt
  common::Time cur_time = world->SimTime();
  double dt = (cur_time - last_time).Double();
  if (last_time + update_period > cur_time) return;

  boost::mutex::scoped_lock scoped_lock(lock);

  // Get Pose/Orientation
  ignition::math::Pose pose = link->WorldPose();

  // get Acceleration and Angular Rates
  // the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
  //accel = myBody->GetRelativeLinearAccel(); // get acceleration in body frame
  ignition::math::Vector3 temp = link->WorldLinearVel(); // get velocity in world frame
  accel = pose.rot.RotateVectorReverse((temp - velocity) / dt);
  velocity = temp;

  // GetRelativeAngularVel() sometimes return nan?
  rate  = link->GetRelativeAngularVel(); // get angular rate in body frame
//  ignition::math::Quaternion delta = pose.rot - orientation;
//  orientation = pose.rot;
//  rate.x = 2.0 * (-orientation.x * delta.w + orientation.w * delta.x + orientation.z * delta.y - orientation.y * delta.z) / dt;
//  rate.y = 2.0 * (-orientation.y * delta.w - orientation.z * delta.x + orientation.w * delta.y + orientation.x * delta.z) / dt;
//  rate.z = 2.0 * (-orientation.z * delta.w + orientation.y * delta.x - orientation.x * delta.y + orientation.w * delta.z) / dt;

  // get Gravity
  gravity       = world->Gravity();
  gravity_body  = orientation.RotateVectorReverse(gravity);
  double gravity_length = gravity.GetLength();
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.x, gravity.y, gravity.z);

  // add gravity vector to body acceleration
  accel = accel  - gravity_body;

  // update sensor models (add drifts, noises, offsets)
  
  accel = accel + accelModel.update(dt);
  rate  = rate  + rateModel.update(dt);
  headingModel.update(dt);
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "Current errors: accel = [%g %g %g], rate = [%g %g %g], heading = %g",
                 accelModel.getCurrentError().x, accelModel.getCurrentError().y, accelModel.getCurrentError().z,
                 rateModel.getCurrentError().x, rateModel.getCurrentError().y, rateModel.getCurrentError().z,
                 headingModel.getCurrentError());

  // apply offset error to orientation (pseudo AHRS) attitude and heading reference system
  double normalization_constant = (gravity_body + accelModel.getCurrentError()).GetLength() * gravity_body.GetLength();
  double cos_alpha = (gravity_body + accelModel.getCurrentError()).Dot(gravity_body)/normalization_constant;
  ignition::math::Vector3 normal_vector(gravity_body.Cross(accelModel.getCurrentError()));
  normal_vector *= sqrt((1 - cos_alpha)/2)/normalization_constant;
  ignition::math::Quaternion attitudeError(sqrt((1 + cos_alpha)/2), normal_vector.x, normal_vector.y, normal_vector.z);
  ignition::math::Quaternion headingError(cos(headingModel.getCurrentError()/2),0,0,sin(headingModel.getCurrentError()/2));
  pose.rot = attitudeError * pose.rot * headingError;

  // copy data into pose message
  imuMsg.header.frame_id = frameId;
  imuMsg.header.stamp.sec = cur_time.sec;
  imuMsg.header.stamp.nsec = cur_time.nsec;

  // orientation quaternion
  imuMsg.orientation.x = pose.rot.x;
  imuMsg.orientation.y = pose.rot.y;
  imuMsg.orientation.z = pose.rot.z;
  imuMsg.orientation.w = pose.rot.w;

  // pass angular rates
  imuMsg.angular_velocity.x    = rate.x;
  imuMsg.angular_velocity.y    = rate.y;
  imuMsg.angular_velocity.z    = rate.z;

  // pass accelerations
  imuMsg.linear_acceleration.x    = accel.x;
  imuMsg.linear_acceleration.y    = accel.y;
  imuMsg.linear_acceleration.z    = accel.z;

  // fill in covariance matrix
  imuMsg.orientation_covariance[8] = headingModel.gaussian_noise*headingModel.gaussian_noise;
  if (gravity_length > 0.0) {
    imuMsg.orientation_covariance[0] = accelModel.gaussian_noise.x*accelModel.gaussian_noise.x/(gravity_length*gravity_length);
    imuMsg.orientation_covariance[4] = accelModel.gaussian_noise.y*accelModel.gaussian_noise.y/(gravity_length*gravity_length);
  } else {
    imuMsg.orientation_covariance[0] = -1;
    imuMsg.orientation_covariance[4] = -1;
  }

  // publish to ros
  pub_.publish(imuMsg);

  // save last time stamp
  last_time = cur_time;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIMU)

} // namespace gazebo
