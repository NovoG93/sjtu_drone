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


#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H

// #define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include "gazebo/common/Plugin.hh"

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <gazebo/gazebo.hh>
#include <sensor_model.h>
#include <sjtu_drone/SetBias.h>


namespace gazebo
{
   class GazeboRosIMU : public ModelPlugin
   {
   public:
      /// \brief Constructor
      GazeboRosIMU();

      /// \brief Destructor
      virtual ~GazeboRosIMU();

   protected:
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Reset();
      virtual void Update();

   private:
      /// \brief The parent World
      physics::WorldPtr world;

      /// \brief The link referred to by this plugin
      physics::LinkPtr link;

      /// \brief pointer to ros node
      ros::NodeHandle* node_handle_;
      ros::Publisher pub_;

      /// \brief ros message
      sensor_msgs::Imu imuMsg;

      /// \brief store link name
      std::string linkName;

      /// \brief frame id
      std::string frameId;

      /// \brief topic name
      std::string topicName;

      /// \brief Sensor models
      SensorModel3 accelModel;
      SensorModel3 rateModel;
      SensorModel headingModel;

      /// \brief A mutex to lock access to fields that are used in message callbacks
      boost::mutex lock;

      /// \brief save last_time
      common::Time last_time;
      common::Time update_period;

      /// \brief save current body/physics state
      math::Quaternion orientation;
      math::Vector3 velocity;
      math::Vector3 accel;
      math::Vector3 rate;
      math::Vector3 gravity;
      math::Vector3 gravity_body;

      /// \brief Gaussian noise generator
      double GaussianKernel(double mu,double sigma);

      /// \brief for setting ROS name space
      std::string robotNamespace;

      /// \brief call back when using service
      bool ServiceCalibrate(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res);
      ros::ServiceServer srv_;
      std::string serviceName;

      /// \brief Bias service callbacks
      bool SetAccelBiasCallback(sjtu_drone::SetBias::Request &req, sjtu_drone::SetBias::Response &res);
      bool SetRateBiasCallback(sjtu_drone::SetBias::Request &req, sjtu_drone::SetBias::Response &res);
      ros::ServiceServer accelBiasService;
      ros::ServiceServer rateBiasService;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
   };
}

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
