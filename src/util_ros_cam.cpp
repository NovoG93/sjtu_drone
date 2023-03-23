/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>
#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>
using namespace boost::placeholders;

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/rendering/Camera.hh>

#include "util_ros_cam.h"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCameraUtils::GazeboRosCameraUtils()
{
  this->last_update_time_ = common::Time(0);
  this->last_info_update_time_ = common::Time(0);
  this->height_ = 0;
  this->width_ = 0;
  this->skip_ = 0;
  this->format_ = "";
  this->initialized_ = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCameraUtils::~GazeboRosCameraUtils()
{
  this->parentSensor_->SetActive(false);
  // this->camera_group_.clear();
  // this->camera_group_.disable();
  this->callback_queue_thread_.join();
  // delete this->node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix,
  double _hack_baseline)
{
  // default Load:
  // provide _camera_name_suffix to prevent LoadThread() creating the ros::NodeHandle with
  //an incomplete this->camera_name_ namespace. There was a race condition when the _camera_name_suffix
  //was appended in this function.
  this->Load(_parent, _sdf, _camera_name_suffix);

  // overwrite hack baseline if specified at load
  // example usage in gazebo_ros_multicamera
  this->hack_baseline_ = _hack_baseline;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::Load(sensors::SensorPtr _parent,
  sdf::ElementPtr _sdf,
  const std::string &_camera_name_suffix)
{
  // Get the world name.
  std::string world_name = _parent->WorldName();

  // Get the world_
  this->world_ = physics::get_world(world_name);

  // save pointers
  this->sdf = _sdf;

  // maintain for one more release for backwards compatibility with
  // pr2_gazebo_plugins
  this->world = this->world_;

  this->robot_namespace_ = _parent->ParentName().substr(0, _parent->ParentName().find("::"));
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  this->image_topic_name_ = "image_raw";
  if (this->sdf->HasElement("imageTopicName"))
    this->image_topic_name_ = this->sdf->Get<std::string>("imageTopicName");

  this->camera_info_topic_name_ = "camera_info";
  if (this->sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ =
      this->sdf->Get<std::string>("cameraInfoTopicName");

  if (!this->sdf->HasElement("cameraName"))
  {
     RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <cameraName>, default to empty");
    this->camera_name_ = "camera";
  }
  else
    this->camera_name_ = this->sdf->Get<std::string>("cameraName");

  if (!this->sdf->HasElement("frameName"))
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <frameName>, defaults to /world");
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("updateRate"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <updateRate>, defaults to unlimited (0).");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = this->sdf->Get<double>("updateRate");

  if (!this->sdf->HasElement("CxPrime"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <CxPrime>, defaults to 0");
    this->cx_prime_ = 0;
  }
  else
    this->cx_prime_ = this->sdf->Get<double>("CxPrime");

  if (!this->sdf->HasElement("Cx"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <Cx>, defaults to 0");
    this->cx_= 0;
  }
  else
    this->cx_ = this->sdf->Get<double>("Cx");

  if (!this->sdf->HasElement("Cy"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <Cy>, defaults to 0");
    this->cy_= 0;
  }
  else
    this->cy_ = this->sdf->Get<double>("Cy");

  if (!this->sdf->HasElement("focalLength"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <focalLength>, defaults to 0");
    this->focal_length_= 0;
  }
  else
    this->focal_length_ = this->sdf->Get<double>("focalLength");

  if (!this->sdf->HasElement("hackBaseline"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <hackBaseline>, defaults to 0");
    this->hack_baseline_= 0;
  }
  else
    this->hack_baseline_ = this->sdf->Get<double>("hackBaseline");

  if (!this->sdf->HasElement("distortionK1"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <distortionK1>, defaults to 0");
    this->distortion_k1_= 0;
  }
  else
    this->distortion_k1_ = this->sdf->Get<double>("distortionK1");

  if (!this->sdf->HasElement("distortionK2"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <distortionK2>, defaults to 0");
    this->distortion_k2_= 0;
  }
  else
    this->distortion_k2_ = this->sdf->Get<double>("distortionK2");

  if (!this->sdf->HasElement("distortionK3"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <distortionK3>, defaults to 0");
    this->distortion_k3_= 0;
  }
  else
    this->distortion_k3_ = this->sdf->Get<double>("distortionK3");

  if (!this->sdf->HasElement("distortionT1"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <distortionT1>, defaults to 0");
    this->distortion_t1_= 0;
  }
  else
    this->distortion_t1_ = this->sdf->Get<double>("distortionT1");

  if (!this->sdf->HasElement("distortionT2"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("DroneCameraController"),  "Camera plugin missing <distortionT2>, defaults to 0");
    this->distortion_t2_= 0;
  }
  else
    this->distortion_t2_ = this->sdf->Get<double>("distortionT2");

  if ((this->distortion_k1_ != 0.0) || (this->distortion_k2_ != 0.0) ||
      (this->distortion_k3_ != 0.0) || (this->distortion_t1_ != 0.0) ||
      (this->distortion_t2_ != 0.0))
  {
    RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "gazebo_ros_camera_ simulation does not support non-zero"
             " distortion parameters right now, your simulation maybe wrong.");
  }

  RCLCPP_INFO(rclcpp::get_logger("DroneSimpleController"), "Initialization finished with ns: %s", this->robot_namespace_.c_str());
  // initialize shared_ptr members
  if (!this->image_connect_count_) this->image_connect_count_ = std::shared_ptr<int>(new int(0));
  if (!this->image_connect_count_lock_) this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
  if (!this->was_active_) this->was_active_ = std::shared_ptr<bool>(new bool(false));

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    std::bind(&GazeboRosCameraUtils::LoadThread, this));
}

event::ConnectionPtr GazeboRosCameraUtils::OnLoad(const boost::function<void()>& load_function)
{
  return load_event_.Connect(load_function);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCameraUtils::LoadThread()
{
  // Exit if no ROS
  if (!rclcpp::ok())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // Sensor generation off by default.  Must do this before advertising the
  // associated ROS topics.
  this->parentSensor_->SetActive(false);

  this->node_handle_ = std::make_shared<rclcpp::Node>(this->camera_name_, this->robot_namespace_);

  this->itnode_ = std::make_shared<image_transport::ImageTransport>(this->node_handle_);

  // resolve tf prefix
  std::string key;
  if(this->node_handle_->has_parameter("tf_prefix")){
    std::string prefix;
    this->node_handle_->get_parameter_or("tf_prefix", prefix);
    this->frame_name_ = prefix + this->frame_name_;
  }


  this->image_pub_ = this->itnode_->advertise(this->image_topic_name_, 2);

  // camera info publish rate will be synchronized to image sensor
  // publish rates.
  // If someone connects to camera_info, sensor will be activated
  // and camera_info will be published alongside image_raw with the
  // same timestamps.  This incurrs additional computational cost when
  // there are subscribers to camera_info, but better mimics behavior
  // of image_pipeline.
  rclcpp::PublisherOptions cio;
  cio.callback_group = this->camera_queue_;
  // auto qos = rclcpp::QoSInitialization::from_rmw(this->qos_profile);
    // rclcpp::PublisherOptions::<sensor_msgs::msg::CameraInfo>(
    // this->camera_info_topic_name_, 2,
    // std::bind(&GazeboRosCameraUtils::ImageConnect, this),
    // std::bind(&GazeboRosCameraUtils::ImageDisconnect, this),
    // rmw_qos_profile_default, &this->camera_queue_);

  

  this->camera_info_pub_ = this->node_handle_->create_publisher<sensor_msgs::msg::CameraInfo>(this->camera_info_topic_name_, 10);
    // qos)//, cio);


  this->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Set Horizontal Field of View
void GazeboRosCameraUtils::SetHFOV(const std_msgs::msg::Float64::SharedPtr hfov)
{
  this->camera_->SetHFOV((ignition::math::Angle)(hfov->data));
}

////////////////////////////////////////////////////////////////////////////////
// Set Update Rate
void GazeboRosCameraUtils::SetUpdateRate(
  const std_msgs::msg::Float64::SharedPtr update_rate)
{
  this->parentSensor_->SetUpdateRate(update_rate->data);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCameraUtils::ImageConnect()
{
  boost::mutex::scoped_lock lock(*this->image_connect_count_lock_);

  // upon first connection, remember if camera was active.
  if ((*this->image_connect_count_) == 0)
    *this->was_active_ = this->parentSensor_->IsActive();

  (*this->image_connect_count_)++;

  this->parentSensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCameraUtils::ImageDisconnect()
{
  boost::mutex::scoped_lock lock(*this->image_connect_count_lock_);

  (*this->image_connect_count_)--;

  // if there are no more subscribers, but camera was active to begin with,
  // leave it active.  Use case:  this could be a multicamera, where
  // each camera shares the same parentSensor_.
  if ((*this->image_connect_count_) <= 0 && !*this->was_active_)
    this->parentSensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCameraUtils::Init()
{
  // prepare to throttle this plugin at the same rate
  // ideally, we should invoke a plugin update when the sensor updates,
  // have to think about how to do that properly later
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  // set buffer size
  if (this->format_ == "L8")
  {
    this->type_ = sensor_msgs::image_encodings::MONO8;
    this->skip_ = 1;
  }
  else if (this->format_ == "R8G8B8")
  {
    this->type_ = sensor_msgs::image_encodings::RGB8;
    this->skip_ = 3;
  }
  else if (this->format_ == "B8G8R8")
  {
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }
  else if (this->format_ == "BAYER_RGGB8")
  {
    RCLCPP_INFO(rclcpp::get_logger("DroneCameraController"),  "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_BGGR8")
  {
    RCLCPP_INFO(rclcpp::get_logger("DroneCameraController"),  "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GBRG8")
  {
    RCLCPP_INFO(rclcpp::get_logger("DroneCameraController"),  "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip_ = 1;
  }
  else if (this->format_ == "BAYER_GRBG8")
  {
    RCLCPP_INFO(rclcpp::get_logger("DroneCameraController"),  "bayer simulation maybe computationally expensive.");
    this->type_ = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip_ = 1;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("DroneCameraController"), "Unsupported Gazebo ImageFormat\n");
    this->type_ = sensor_msgs::image_encodings::BGR8;
    this->skip_ = 3;
  }

  /// Compute camera_ parameters if set to 0
  if (this->cx_prime_ == 0)
    this->cx_prime_ = (static_cast<double>(this->width_) + 1.0) /2.0;
  if (this->cx_ == 0)
    this->cx_ = (static_cast<double>(this->width_) + 1.0) /2.0;
  if (this->cy_ == 0)
    this->cy_ = (static_cast<double>(this->height_) + 1.0) /2.0;


  double computed_focal_length =
    (static_cast<double>(this->width_)) /
    (2.0 * tan(this->camera_->HFOV().Radian() / 2.0));

  if (this->focal_length_ == 0)
  {
    this->focal_length_ = computed_focal_length;
  }
  else
  {
    // check against float precision
    if (!ignition::math::v6::equal(this->focal_length_, computed_focal_length))
    {
      RCLCPP_WARN(rclcpp::get_logger("DroneCameraController"), "The <focal_length>[%f] you have provided for camera_ [%s]"
               " is inconsistent with specified image_width [%d] and"
               " HFOV [%f].   Please double check to see that"
               " focal_length = width_ / (2.0 * tan(HFOV/2.0)),"
               " the explected focal_lengtth value is [%f],"
               " please update your camera_ model description accordingly.",
                this->focal_length_, this->parentSensor_->Name().c_str(),
                this->width_, this->camera_->HFOV().Radian(),
                computed_focal_length);
    }
  }


  // start custom queue for camera_
  this->callback_queue_thread_ = boost::thread(
    std::bind(&GazeboRosCameraUtils::CameraQueueThread, this));

  load_event_();
  this->initialized_ = true;
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src,
  common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PutCameraData(_src);
}

void GazeboRosCameraUtils::PutCameraData(const unsigned char *_src)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  /// don't bother if there are no subscribers
  if ((*this->image_connect_count_) > 0)
  {
    boost::mutex::scoped_lock lock(this->lock_);

    // copy data into image
    this->image_msg_.header.frame_id = this->frame_name_;
    this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
    this->image_msg_.header.stamp.nanosec = this->sensor_update_time_.nsec;

    // copy from src to image_msg_
    sensor_msgs::fillImage(this->image_msg_, this->type_, this->height_, this->width_,
        this->skip_*this->width_, reinterpret_cast<const void*>(_src));

    // publish to ros
    this->image_pub_.publish(this->image_msg_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::PublishCameraInfo(common::Time &last_update_time)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->sensor_update_time_ = last_update_time;
  this->PublishCameraInfo();
}

void GazeboRosCameraUtils::PublishCameraInfo()
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  if (this->camera_info_pub_->get_subscription_count() > 0)
  {
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    common::Time cur_time = this->world_->SimTime();
    if (cur_time - this->last_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->camera_info_pub_);
      this->last_info_update_time_ = cur_time;
    }
  }
}

void GazeboRosCameraUtils::PublishCameraInfo(
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_)
{
  sensor_msgs::msg::CameraInfo camera_info_msg;
  // fill CameraInfo
  camera_info_msg.header.frame_id = this->frame_name_;

  camera_info_msg.header.stamp.sec = this->sensor_update_time_.sec;
  camera_info_msg.header.stamp.nanosec = this->sensor_update_time_.nsec;
  camera_info_msg.height = this->height_;
  camera_info_msg.width  = this->width_;
  // distortion
  camera_info_msg.d[0] = this->distortion_k1_;
  camera_info_msg.d[1] = this->distortion_k2_;
  camera_info_msg.d[2] = this->distortion_k3_;
  camera_info_msg.d[3] = this->distortion_t1_;
  camera_info_msg.d[4] = this->distortion_t2_;
  // original camera_ matrix
  camera_info_msg.k[0] = this->focal_length_;
  camera_info_msg.k[1] = 0.0;
  camera_info_msg.k[2] = this->cx_;
  camera_info_msg.k[3] = 0.0;
  camera_info_msg.k[4] = this->focal_length_;
  camera_info_msg.k[5] = this->cy_;
  camera_info_msg.k[6] = 0.0;
  camera_info_msg.k[7] = 0.0;
  camera_info_msg.k[8] = 1.0;
  // rectification
  camera_info_msg.r[0] = 1.0;
  camera_info_msg.r[1] = 0.0;
  camera_info_msg.r[2] = 0.0;
  camera_info_msg.r[3] = 0.0;
  camera_info_msg.r[4] = 1.0;
  camera_info_msg.r[5] = 0.0;
  camera_info_msg.r[6] = 0.0;
  camera_info_msg.r[7] = 0.0;
  camera_info_msg.r[8] = 1.0;
  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg.p[0] = this->focal_length_;
  camera_info_msg.p[1] = 0.0;
  camera_info_msg.p[2] = this->cx_;
  camera_info_msg.p[3] = -this->focal_length_ * this->hack_baseline_;
  camera_info_msg.p[4] = 0.0;
  camera_info_msg.p[5] = this->focal_length_;
  camera_info_msg.p[6] = this->cy_;
  camera_info_msg.p[7] = 0.0;
  camera_info_msg.p[8] = 0.0;
  camera_info_msg.p[9] = 0.0;
  camera_info_msg.p[10] = 1.0;
  camera_info_msg.p[11] = 0.0;

  camera_info_pub_->publish(camera_info_msg);
}


////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
void GazeboRosCameraUtils::CameraQueueThread()
{
  auto timeout = std::chrono::microseconds(100);
  executor.add_callback_group(camera_queue_, this->node_handle_->get_node_base_interface());

  // while (this->node_handle_->ok())
  while (rclcpp::ok())
  {
    /// take care of callback queue
    // executor.spin_some(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::duration<double>(timeout)));  }
    executor.spin_some(timeout);  
  }
}
}
