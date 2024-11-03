/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

#include "openni2_camera/openni2_driver.h"
#include "openni2_camera/openni2_exception.h"

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>

namespace openni2_wrapper
{

using namespace std::placeholders;
using namespace std::chrono_literals;

OpenNI2Driver::OpenNI2Driver(const rclcpp::NodeOptions & node_options) :
    Node("openni2_camera", node_options),
    device_manager_(OpenNI2DeviceManager::getSingelton()),
    data_skip_ir_counter_(0),
    data_skip_color_counter_(0),
    data_skip_depth_counter_ (0),
    ir_subscribers_(false),
    color_subscribers_(false),
    depth_subscribers_(false),
    depth_raw_subscribers_(false),
    enable_reconnect_(false),
    serialnumber_as_name_(false)
{
  // Declare parameters
  depth_ir_offset_x_ = declare_parameter<double>("depth_ir_offset_x", 5.0);
  depth_ir_offset_y_ = declare_parameter<double>("depth_ir_offset_y", 4.0);

  z_offset_mm_ = declare_parameter<int>("z_offset_mm", 0);
  z_scaling_ = declare_parameter<double>("z_scaling", 1.0);

  ir_time_offset_ = declare_parameter<double>("ir_time_offset", -0.033);
  color_time_offset_ = declare_parameter<double>("color_time_offset", -0.033);
  depth_time_offset_ = declare_parameter<double>("depth_time_offset", -0.033);

  depth_registration_ = declare_parameter<bool>("depth_registration", true);
  color_depth_synchronization_ = declare_parameter<bool>("color_depth_synchronization", false);
  auto_exposure_ = declare_parameter<bool>("auto_exposure", true);
  auto_white_balance_ = declare_parameter<bool>("auto_white_balance", true);
  use_device_time_ = declare_parameter<bool>("use_device_time", true);
  exposure_ = declare_parameter<int>("exposure", 0);
  data_skip_ = declare_parameter<int>("data_skip", 0) + 1;
  enable_reconnect_ = declare_parameter<bool>("enable_reconnect", true);

  ir_frame_id_ = declare_parameter<std::string>("ir_frame_id", "openni_ir_optical_frame");
  color_frame_id_ = declare_parameter<std::string>("rgb_frame_id", "openni_rgb_optical_frame");
  depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", "openni_depth_optical_frame");

  color_info_url_ = declare_parameter<std::string>("rgb_camera_info_url", "");
  ir_info_url_ = declare_parameter<std::string>("depth_camera_info_url", "");

  genVideoModeTableMap();
  std::string mode = declare_parameter<std::string>("ir_mode", "VGA_30Hz");
  if (!lookupVideoMode(mode, ir_video_mode_))
  {
    RCLCPP_ERROR(this->get_logger(), "Undefined IR video mode");
  }

  mode = declare_parameter<std::string>("color_mode", "VGA_30Hz");
  if (!lookupVideoMode(mode, color_video_mode_))
  {
    RCLCPP_ERROR(this->get_logger(), "Undefined color video mode");
  }

  mode = declare_parameter<std::string>("depth_mode", "VGA_30Hz");
  if (!lookupVideoMode(mode, depth_video_mode_))
  {
    RCLCPP_ERROR(this->get_logger(), "Undefined color video mode");
  }

  ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
  color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
  depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

  device_id_ = declare_parameter<std::string>("device_id", "#1");
  if (device_id_ == "#1")
  {
    RCLCPP_WARN(this->get_logger(), "device_id is not set! Using first device.");
  }

  if (enable_reconnect_)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Reconnect has been enabled, only one camera "
                                            << "should be plugged into each bus");
  }
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Reconnect has been disabled");
  }

  initialized_ = false;
  timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&OpenNI2Driver::periodic, this));
}

void OpenNI2Driver::periodic()
{
  if (!initialized_)
  {
    initDevice();
    advertiseROSTopics();
    applyConfigToOpenNIDevice();

    // Register parameter callback
    parameters_callback_ = \
    this->add_on_set_parameters_callback(std::bind(&OpenNI2Driver::paramCb, this, std::placeholders::_1));
    initialized_ = true;
  }

  if (enable_reconnect_)
    monitorConnection();
}

void OpenNI2Driver::advertiseROSTopics()
{
  // Advertise all published topics
  image_transport::ImageTransport it(this->shared_from_this());

  // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
  // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
  // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
  // the depth generator.
  std::lock_guard<std::mutex> lock(connect_mutex_);

  // Asus Xtion PRO does not have an RGB camera
  if (device_->hasColorSensor())
  {
    // Create publisher with connect callback
    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback =
      [this](rclcpp::MatchedInfo&)
      {
        colorConnectCb();
      };
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;
    pub_color_ = image_transport::create_camera_publisher(this, "rgb/image_raw", custom_qos, pub_options);
  }

  if (device_->hasIRSensor())
  {
    // Create publisher with connect callback
    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback =
      [this](rclcpp::MatchedInfo&)
      {
        irConnectCb();
      };
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;
    pub_ir_ = image_transport::create_camera_publisher(this, "ir/image_raw", custom_qos, pub_options);
  }

  if (device_->hasDepthSensor())
  {
    // Create publisher with connect callback
    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback =
      [this](rclcpp::MatchedInfo&)
      {
        depthConnectCb();
      };
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;
    pub_depth_raw_ = image_transport::create_camera_publisher(this, "depth_raw/image", custom_qos, pub_options);
    pub_depth_ = image_transport::create_camera_publisher(this, "depth/image", custom_qos, pub_options);
    pub_projector_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("projector/camera_info", 1);
  }

  ////////// CAMERA INFO MANAGER

  // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
  // camera_info_manager substitutes this for ${NAME} in the URL.
  std::string serial_number;
  if (serialnumber_as_name_)
    serial_number = device_manager_->getSerial(device_->getUri());
  else
    serial_number = device_->getStringID();

  std::string color_name, ir_name;
  color_name = "rgb_"   + serial_number;
  ir_name  = "depth_" + serial_number;

  // Load the saved calibrations, if they exist
  color_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, color_name, color_info_url_);
  ir_info_manager_  = std::make_shared<camera_info_manager::CameraInfoManager>(this,  ir_name,  ir_info_url_);

  get_serial_server = this->create_service<openni2_camera::srv::GetSerial>(
    "get_serial",
    std::bind(&OpenNI2Driver::getSerialCb, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void OpenNI2Driver::getSerialCb(const std::shared_ptr<openni2_camera::srv::GetSerial::Request> request,
                                std::shared_ptr<openni2_camera::srv::GetSerial::Response> response)
{
  response->serial = device_manager_->getSerial(device_->getUri());
}

rcl_interfaces::msg::SetParametersResult OpenNI2Driver::paramCb(
  const std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();

  // Assume success until we fail
  result.successful = true;

  // Apply parameters
  for (const auto & param : parameters)
  {
    if (param.get_name() == "z_offset_mm")
    {
      z_offset_mm_ = param.as_int();
    }
    else if (param.get_name() == "z_scaling")
    {
      z_scaling_ = param.as_double();
    }
    else if (param.get_name() == "ir_time_offset")
    {
      ir_time_offset_ = param.as_double();
    }
    else if (param.get_name() == "color_time_offset")
    {
      color_time_offset_ = param.as_double();
    }
    else if (param.get_name() == "depth_time_offset")
    {
      depth_time_offset_ = param.as_double();
    }
    else if (param.get_name() == "auto_exposure")
    {
      auto_exposure_ = param.as_bool();
    }
    else if (param.get_name() == "auto_white_balance")
    {
      auto_white_balance_ = param.as_bool();
    }
    else if (param.get_name() == "exposure")
    {
      exposure_ = param.as_int();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Parameter %s is not settable", param.get_name().c_str());
      result.successful = false;
    }
  }

  applyConfigToOpenNIDevice();
  return result;
}

void OpenNI2Driver::setIRVideoMode(const OpenNI2VideoMode& ir_video_mode)
{
  if (device_->isIRVideoModeSupported(ir_video_mode))
  {
    if (ir_video_mode != device_->getIRVideoMode())
    {
      device_->setIRVideoMode(ir_video_mode);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unsupported IR video mode - " << ir_video_mode);
  }
}
void OpenNI2Driver::setColorVideoMode(const OpenNI2VideoMode& color_video_mode)
{
  if (device_->isColorVideoModeSupported(color_video_mode))
  {
    if (color_video_mode != device_->getColorVideoMode())
    {
      device_->setColorVideoMode(color_video_mode);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unsupported color video mode - " << color_video_mode);
  }
}
void OpenNI2Driver::setDepthVideoMode(const OpenNI2VideoMode& depth_video_mode)
{
  if (device_->isDepthVideoModeSupported(depth_video_mode))
  {
    if (depth_video_mode != device_->getDepthVideoMode())
    {
      device_->setDepthVideoMode(depth_video_mode);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unsupported depth video mode - " << depth_video_mode);
  }
}

void OpenNI2Driver::applyConfigToOpenNIDevice()
{
  data_skip_ir_counter_ = 0;
  data_skip_color_counter_= 0;
  data_skip_depth_counter_ = 0;

  setIRVideoMode(ir_video_mode_);
  setColorVideoMode(color_video_mode_);
  setDepthVideoMode(depth_video_mode_);

  if (device_->isImageRegistrationModeSupported())
  {
    try
    {
      device_->setImageRegistrationMode(depth_registration_);
    }
    catch (const OpenNI2Exception& exception)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not set image registration. Reason: %s", exception.what());
    }
  }

  try
  {
    device_->setDepthColorSync(color_depth_synchronization_);
  }
  catch (const OpenNI2Exception& exception)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not set color depth synchronization. Reason: %s", exception.what());
  }

  try
  {
    device_->setAutoExposure(auto_exposure_);
  }
  catch (const OpenNI2Exception& exception)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not set auto exposure. Reason: %s", exception.what());
  }

  try
  {
    device_->setAutoWhiteBalance(auto_white_balance_);
  }
  catch (const OpenNI2Exception& exception)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not set auto white balance. Reason: %s", exception.what());
  }

  // Workaound for https://github.com/ros-drivers/openni2_camera/issues/51
  // This is only needed when any of the 3 setting change.  For simplicity
  // this check is always performed and exposure set.
  if( (!auto_exposure_ && !auto_white_balance_) && exposure_ != 0 )
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Forcing exposure set, when auto exposure/white balance disabled");
    forceSetExposure();
  }
  else
  {
    // Setting the exposure the old way, although this should not have an effect
    try
    {
      device_->setExposure(exposure_);
    }
    catch (const OpenNI2Exception& exception)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not set exposure. Reason: %s", exception.what());
    }
  }

  device_->setUseDeviceTimer(use_device_time_);
}

void OpenNI2Driver::forceSetExposure()
{
  int current_exposure_ = device_->getExposure();
  try
  {
    if (current_exposure_ == exposure_)
    {
      if (exposure_ < 254)
      {
        device_->setExposure(exposure_ + 1);
      }
      else
      {
        device_->setExposure(exposure_ - 1);
      }
    }
    device_->setExposure(exposure_);
  }
  catch (const OpenNI2Exception& exception)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not set exposure. Reason: %s", exception.what());
  }
}

void OpenNI2Driver::colorConnectCb()
{
  if (!device_)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Callback in " << __FUNCTION__ <<  "failed due to null device");
    return;
  }
  std::lock_guard<std::mutex> lock(connect_mutex_);

  color_subscribers_ = pub_color_.getNumSubscribers() > 0;

  if (color_subscribers_ && !device_->isColorStreamStarted())
  {
    // Can't stream IR and RGB at the same time. Give RGB preference.
    if (device_->isIRStreamStarted())
    {
      RCLCPP_ERROR(this->get_logger(), "Cannot stream RGB and IR at the same time. Streaming RGB only.");
      RCLCPP_INFO(this->get_logger(), "Stopping IR stream.");
      device_->stopIRStream();
    }

    device_->setColorFrameCallback(std::bind(&OpenNI2Driver::newColorFrameCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Starting color stream.");
    device_->startColorStream();

    // Workaound for https://github.com/ros-drivers/openni2_camera/issues/51
    if (exposure_ != 0)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Exposure is set to " << exposure_ << ", forcing on color stream start");
        //delay for stream to start, before setting exposure
      std::this_thread::sleep_for(100ms);
      forceSetExposure();
    }
  }
  else if (!color_subscribers_ && device_->isColorStreamStarted())
  {
    RCLCPP_INFO(this->get_logger(), "Stopping color stream.");
    device_->stopColorStream();

    // Start IR if it's been blocked on RGB subscribers
    bool need_ir = pub_ir_.getNumSubscribers() > 0;
    if (need_ir && !device_->isIRStreamStarted())
    {
      device_->setIRFrameCallback(std::bind(&OpenNI2Driver::newIRFrameCallback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Starting IR stream.");
      device_->startIRStream();
    }
  }
}

void OpenNI2Driver::depthConnectCb()
{
  if( !device_ )
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Callback in " << __FUNCTION__ <<  "failed due to null device");
    return;
  }
  std::lock_guard<std::mutex> lock(connect_mutex_);

  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  depth_raw_subscribers_ = pub_depth_raw_.getNumSubscribers() > 0;
  projector_info_subscribers_ = pub_projector_info_->get_subscription_count() > 0;

  bool need_depth = depth_subscribers_ || depth_raw_subscribers_;

  if (need_depth && !device_->isDepthStreamStarted())
  {
    device_->setDepthFrameCallback(std::bind(&OpenNI2Driver::newDepthFrameCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Starting depth stream.");
    device_->startDepthStream();
  }
  else if (!need_depth && device_->isDepthStreamStarted())
  {
    RCLCPP_INFO(this->get_logger(), "Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void OpenNI2Driver::irConnectCb()
{
  if( !device_ )
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Callback in " << __FUNCTION__ <<  "failed due to null device");
    return;
  }
  std::lock_guard<std::mutex> lock(connect_mutex_);

  ir_subscribers_ = this->count_subscribers("ir/image") > 0 ||
                    this->count_subscribers("ir/camera_info") > 0;

  if (ir_subscribers_ && !device_->isIRStreamStarted())
  {
    // Can't stream IR and RGB at the same time
    if (device_->isColorStreamStarted())
    {
      RCLCPP_ERROR(this->get_logger(), "Cannot stream RGB and IR at the same time. Streaming RGB only.");
    }
    else
    {
      device_->setIRFrameCallback(std::bind(&OpenNI2Driver::newIRFrameCallback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Starting IR stream.");
      device_->startIRStream();
    }
  }
  else if (!ir_subscribers_ && device_->isIRStreamStarted())
  {
    RCLCPP_INFO(this->get_logger(), "Stopping IR stream.");
    device_->stopIRStream();
  }
}

void OpenNI2Driver::newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  if (!rclcpp::ok())
  {
    // Don't publish to invalid publishers
    return;
  }

  if ((++data_skip_ir_counter_)%data_skip_==0)
  {
    data_skip_ir_counter_ = 0;

    if (ir_subscribers_)
    {
      image->header.frame_id = ir_frame_id_;
      image->header.stamp = rclcpp::Time(image->header.stamp) + rclcpp::Duration::from_seconds(ir_time_offset_);

      pub_ir_.publish(image, getIRCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void OpenNI2Driver::newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  if (!rclcpp::ok())
  {
    // Don't publish to invalid publishers
    return;
  }

  if ((++data_skip_color_counter_)%data_skip_==0)
  {
    data_skip_color_counter_ = 0;

    if (color_subscribers_)
    {
      image->header.frame_id = color_frame_id_;
      image->header.stamp = rclcpp::Time(image->header.stamp) + rclcpp::Duration::from_seconds(color_time_offset_);

      pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void OpenNI2Driver::newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  if (!rclcpp::ok())
  {
    // Don't publish to invalid publishers
    return;
  }

  if ((++data_skip_depth_counter_)%data_skip_==0)
  {

    data_skip_depth_counter_ = 0;

    if (depth_raw_subscribers_||depth_subscribers_||projector_info_subscribers_)
    {
      image->header.stamp = rclcpp::Time(image->header.stamp) + rclcpp::Duration::from_seconds(depth_time_offset_);

      if (z_offset_mm_ != 0)
      {
        uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
        for (unsigned int i = 0; i < image->width * image->height; ++i)
          if (data[i] != 0)
                data[i] += z_offset_mm_;
      }

      if (fabs(z_scaling_ - 1.0) > 1e-6)
      {
        uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
        for (unsigned int i = 0; i < image->width * image->height; ++i)
          if (data[i] != 0)
                data[i] = static_cast<uint16_t>(data[i] * z_scaling_);
      }

      sensor_msgs::msg::CameraInfo::SharedPtr cam_info;

      if (depth_registration_)
      {
        image->header.frame_id = color_frame_id_;
        cam_info = getColorCameraInfo(image->width,image->height, image->header.stamp);
      } else
      {
        image->header.frame_id = depth_frame_id_;
        cam_info = getDepthCameraInfo(image->width,image->height, image->header.stamp);
      }

      if (depth_raw_subscribers_)
      {
        pub_depth_raw_.publish(image, cam_info);
      }

      if (depth_subscribers_ )
      {
        sensor_msgs::msg::Image::ConstSharedPtr floating_point_image = rawToFloatingPointConversion(image);
        pub_depth_.publish(floating_point_image, cam_info);
      }

      // Projector "info" probably only useful for working with disparity images
      if (projector_info_subscribers_)
      {
        pub_projector_info_->publish(*getProjectorCameraInfo(image->width, image->height, image->header.stamp));
      }
    }
  }
}

// Methods to get calibration parameters for the various cameras
sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getDefaultCameraInfo(int width, int height, double f) const
{
  sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->d.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->k.fill(0.0);
  info->k[0] = info->k[4] = f;
  info->k[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->k[5] = (width * (3./8.)) - 0.5;
  info->k[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->r.fill(0.0);
  info->r[0] = info->r[4] = info->r[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->p.fill(0.0);
  info->p[0]  = info->p[5] = f; // fx, fy
  info->p[2]  = info->k[2];     // cx
  info->p[6]  = info->k[5];     // cy
  info->p[10] = 1.0;

  return info;
}

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getColorCameraInfo(int width, int height, rclcpp::Time time) const
{
  sensor_msgs::msg::CameraInfo::SharedPtr info;

  if (color_info_manager_->isCalibrated())
  {
    info = std::make_shared<sensor_msgs::msg::CameraInfo>(color_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      RCLCPP_WARN_ONCE(this->get_logger(), "Image resolution doesn't match calibration of the RGB camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getColorFocalLength(height));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, device_->getColorFocalLength(height));
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = color_frame_id_;

  return info;
}


sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getIRCameraInfo(int width, int height, rclcpp::Time time) const
{
  sensor_msgs::msg::CameraInfo::SharedPtr info;

  if (ir_info_manager_->isCalibrated())
  {
    info = std::make_shared<sensor_msgs::msg::CameraInfo>(ir_info_manager_->getCameraInfo());
    if ( info->width != width )
    {
      // Use uncalibrated values
      RCLCPP_WARN_ONCE(this->get_logger(), "Image resolution doesn't match calibration of the IR camera. Using default parameters.");
      info = getDefaultCameraInfo(width, height, device_->getIRFocalLength(height));
    }
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(width, height, device_->getDepthFocalLength(height));
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getDepthCameraInfo(int width, int height, rclcpp::Time time) const
{
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See http://www.ros.org/wiki/kinect_calibration/technical

  double scaling = (double)width / 640;

  sensor_msgs::msg::CameraInfo::SharedPtr info = getIRCameraInfo(width, height, time);
  info->k[2] -= depth_ir_offset_x_*scaling; // cx
  info->k[5] -= depth_ir_offset_y_*scaling; // cy
  info->p[2] -= depth_ir_offset_x_*scaling; // cx
  info->p[6] -= depth_ir_offset_y_*scaling; // cy

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getProjectorCameraInfo(int width, int height, rclcpp::Time time) const
{
  // The projector info is simply the depth info with the baseline encoded in the P matrix.
  // It's only purpose is to be the "right" camera info to the depth camera's "left" for
  // processing disparity images.
  sensor_msgs::msg::CameraInfo::SharedPtr info = getDepthCameraInfo(width, height, time);
  // Tx = -baseline * fx
  info->p[3] = -device_->getBaseline() * info->p[0];
  return info;
}

std::string OpenNI2Driver::resolveDeviceURI(const std::string& device_id)
{
  // retrieve available device URIs, they look like this: "1d27/0601@1/5"
  // which is <vendor ID>/<product ID>@<bus number>/<device number>
  std::shared_ptr<std::vector<std::string> > available_device_URIs =
    device_manager_->getConnectedDeviceURIs();

  // look for '#<number>' format
  if (device_id.size() > 1 && device_id[0] == '#')
  {
    std::istringstream device_number_str(device_id.substr(1));
    int device_number;
    device_number_str >> device_number;
    int device_index = device_number - 1; // #1 refers to first device
    if (device_index >= available_device_URIs->size() || device_index < 0)
    {
      THROW_OPENNI_EXCEPTION(
          "Invalid device number %i, there are %zu devices connected.",
          device_number, available_device_URIs->size());
    }
    else
    {
      return available_device_URIs->at(device_index);
    }
  }
  // look for '<bus>@<number>' format
  //   <bus>    is usb bus id, typically start at 1
  //   <number> is the device number, for consistency with openni_camera, these start at 1
  //               although 0 specifies "any device on this bus"
  else if (device_id.size() > 1 && device_id.find('@') != std::string::npos && device_id.find('/') == std::string::npos)
  {
    // get index of @ character
    size_t index = device_id.find('@');
    if (index <= 0)
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give the bus number before the @.",
        device_id.c_str());
    }
    if (index >= device_id.size() - 1)
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give the device number after the @, specify 0 for any device on this bus",
        device_id.c_str());
    }

    // pull out device number on bus
    std::istringstream device_number_str(device_id.substr(index+1));
    int device_number;
    device_number_str >> device_number;

    // reorder to @<bus>
    std::string bus = device_id.substr(0, index);
    bus.insert(0, "@");

    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(bus) != std::string::npos)
      {
        // this matches our bus, check device number
        std::ostringstream ss;
        ss << bus << '/' << device_number;
        if (device_number == 0 || s.find(ss.str()) != std::string::npos)
          return s;
      }
    }

    THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
  }
  else
  {
    // check if the device id given matches a serial number of a connected device
    for(std::vector<std::string>::const_iterator it = available_device_URIs->begin();
        it != available_device_URIs->end(); ++it)
    {
      try {
        std::string serial = device_manager_->getSerial(*it);
        if (serial.size() > 0 && device_id == serial)
          return *it;
      }
      catch (const OpenNI2Exception& exception)
      {
        RCLCPP_WARN(this->get_logger(), "Could not query serial number of device \"%s\":", exception.what());
      }
    }

    // everything else is treated as part of the device_URI
    bool match_found = false;
    std::string matched_uri;
    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(device_id) != std::string::npos)
      {
        if (!match_found)
        {
          matched_uri = s;
          match_found = true;
        }
        else
        {
          // more than one match
          THROW_OPENNI_EXCEPTION("Two devices match the given device id '%s': %s and %s.", device_id.c_str(), matched_uri.c_str(), s.c_str());
        }
      }
    }
    if (match_found)
      return matched_uri;
  }
  return "INVALID";
}

void OpenNI2Driver::initDevice()
{
  while (rclcpp::ok() && !device_)
  {
    try
    {
      std::string device_URI = resolveDeviceURI(device_id_);
      device_ = device_manager_->getDevice(device_URI, this);
      bus_id_ = extractBusID(device_->getUri() );
    }
    catch (const OpenNI2Exception& exception)
    {
      if (!device_)
      {
        RCLCPP_INFO(this->get_logger(), "No matching device found.... waiting for devices. Reason: %s", exception.what());
        std::this_thread::sleep_for(3s);
        continue;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Could not retrieve device. Reason: %s", exception.what());
        exit(-1);
      }
    }
  }

  while (rclcpp::ok() && !device_->isValid())
  {
    RCLCPP_ERROR(this->get_logger(), "Waiting for device initialization..");
    std::this_thread::sleep_for(100ms);
  }
}

int OpenNI2Driver::extractBusID(const std::string& uri) const
{
  // URI format is <vendor ID>/<product ID>@<bus number>/<device number>
  unsigned first = uri.find('@');
  unsigned last = uri.find('/', first);
  std::string bus_id = uri.substr (first+1,last-first-1);
  int rtn = atoi(bus_id.c_str());
  return rtn;
}

bool OpenNI2Driver::isConnected() const
{
  // TODO: The current isConnected logic assumes that there is only one sensor
  // on the bus of interest.  In the future, we could compare serial numbers
  // to make certain the same camera as been re-connected.
  std::shared_ptr<std::vector<std::string> > list =
      device_manager_->getConnectedDeviceURIs();
  for (std::size_t i = 0; i != list->size(); ++i)
  {
    int uri_bus_id = extractBusID( list->at(i) );
    if( uri_bus_id == bus_id_ )
    {
      return true;
    }
  }
  return false;
}

void OpenNI2Driver::monitorConnection()
{
  // If the connection is lost, clean up the device.  If connected
  // and the devices is not initialized, then initialize.
  if( isConnected() )
  {
    if( !device_ )
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Detected re-connect...attempting reinit");
      try
      {
        {
        std::lock_guard<std::mutex> lock(connect_mutex_);
        std::string device_URI = resolveDeviceURI(device_id_);
        device_ = device_manager_->getDevice(device_URI, this);
        bus_id_ = extractBusID(device_->getUri() );
        while (rclcpp::ok() && !device_->isValid())
        {
          RCLCPP_INFO(this->get_logger(), "Waiting for device initialization, before configuring and restarting publishers");
          std::this_thread::sleep_for(100ms);
        }
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Re-applying configuration to camera on re-init");
        //config_init_ = false;
        applyConfigToOpenNIDevice();

        // The color stream must be started in order to adjust the exposure white
        // balance.
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting color stream to adjust camera");
        colorConnectCb();

        // If auto exposure/white balance is disabled, then the rbg image won't
        // be adjusted properly.  This is a work around for now, but the final
        // implimentation should only allow reconnection when auto exposure and
        // white balance are disabled, and FIXED exposure is used instead.
        if((!auto_exposure_ && !auto_white_balance_ ) && exposure_ == 0)
        {
          RCLCPP_WARN_STREAM(this->get_logger(), "Reconnection should not be enabled if auto expousre"
                          << "/white balance are disabled.  Temporarily working"
                          << " around this issue");
          RCLCPP_WARN_STREAM(this->get_logger(), "Toggling exposure and white balance to auto on re-connect"
                          << ", otherwise image will be very dark");
          device_->setAutoExposure(true);
          device_->setAutoWhiteBalance(true);
          RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for color camera to come up and adjust");
          // It takes about 2.5 seconds for the camera to adjust
          std::this_thread::sleep_for(2500ms);
          RCLCPP_WARN_STREAM(this->get_logger(), "Resetting auto exposure and white balance to previous values");
          device_->setAutoExposure(auto_exposure_);
          device_->setAutoWhiteBalance(auto_white_balance_);
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Restarting publishers, if needed");
        irConnectCb();
        depthConnectCb();
        RCLCPP_INFO_STREAM(this->get_logger(), "Done re-initializing cameras");
      }

      catch (const OpenNI2Exception& exception)
      {
        if (!device_)
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "Failed to re-initialize device on bus: " << bus_id_
                          << ", reason: " << exception.what());
        }
      }
    }
  }
  else if( device_ )
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Detected loss of connection.  Stopping all streams and resetting device");
    device_->stopAllStreams();
    device_.reset();
  }
}


void OpenNI2Driver::genVideoModeTableMap()
{
  video_modes_lookup_.clear();

  OpenNI2VideoMode video_mode;

  // SXGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_["SXGA_30Hz"] = video_mode;

  // SXGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_["SXGA_15hz"] = video_mode;

  // XGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_["XGA_30Hz"] = video_mode;

  // XGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_["XGA_15Hz"] = video_mode;

  // VGA_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_["VGA_30Hz"] = video_mode;

  // VGA_25Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_["VGA_25Hz"] = video_mode;

  // QVGA_25Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_["QVGA_25Hz"] = video_mode;

  // QVGA_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_["QVGA_30Hz"] = video_mode;

  // QVGA_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_["QVGA_60Hz"] = video_mode;

  // QQVGA_25Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_["QVGA_25Hz"] = video_mode;

  // QQVGA_30Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_["QQVGA_30Hz"] = video_mode;

  // QQVGA_60Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_["QQVGA_60Hz"] = video_mode;
}

bool OpenNI2Driver::lookupVideoMode(const std::string& mode, OpenNI2VideoMode& video_mode)
{
  auto it = video_modes_lookup_.find(mode);
  if (it != video_modes_lookup_.end())
  {
    video_mode = it->second;
    return true;
  }

  return false;
}

sensor_msgs::msg::Image::ConstSharedPtr OpenNI2Driver::rawToFloatingPointConversion(sensor_msgs::msg::Image::ConstSharedPtr raw_image)
{
  static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  sensor_msgs::msg::Image::SharedPtr new_image = std::make_shared<sensor_msgs::msg::Image>();

  new_image->header = raw_image->header;
  new_image->width = raw_image->width;
  new_image->height = raw_image->height;
  new_image->is_bigendian = 0;
  new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  new_image->step = sizeof(float)*raw_image->width;

  std::size_t data_size = new_image->width*new_image->height;
  new_image->data.resize(data_size*sizeof(float));

  const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
  float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

  for (std::size_t i = 0; i<data_size; ++i, ++in_ptr, ++out_ptr)
  {
    if (*in_ptr==0 || *in_ptr==0x7FF)
    {
      *out_ptr = bad_point;
    } else
    {
      *out_ptr = static_cast<float>(*in_ptr)/1000.0f;
    }
  }

  return new_image;
}

}  // namespace openni2_wrapper

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(openni2_wrapper::OpenNI2Driver)
