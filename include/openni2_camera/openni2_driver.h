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

#ifndef OPENNI2_DRIVER_H
#define OPENNI2_DRIVER_H

#include <functional>

#include <sensor_msgs/msg/image.hpp>

#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <string>
#include <vector>

#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_device.h"
#include "openni2_camera/openni2_video_mode.h"
#include "openni2_camera/srv/get_serial.hpp"

#include <rclcpp/rclcpp.hpp>

namespace openni2_wrapper
{

class OpenNI2Driver : public rclcpp::Node
{
public:
  OpenNI2Driver(const rclcpp::NodeOptions & node_options);

private:
  void newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image);
  void newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image);

  // Methods to get calibration parameters for the various cameras
  sensor_msgs::msg::CameraInfo::SharedPtr getDefaultCameraInfo(int width, int height, double f) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getColorCameraInfo(int width, int height, rclcpp::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getIRCameraInfo(int width, int height, rclcpp::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getDepthCameraInfo(int width, int height, rclcpp::Time time) const;
  sensor_msgs::msg::CameraInfo::SharedPtr getProjectorCameraInfo(int width, int height, rclcpp::Time time) const;

  // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
  std::string resolveDeviceURI(const std::string& device_id);
  void initDevice();

  void advertiseROSTopics();

  // TODO: this is hack around two issues
  //   First, subscription callbacks do not yet exist in ROS2
  //   Second, we can't initialize topics in the constructor (shared_from_this doesn't work yet)
  void periodic();
  bool initialized_;

  void monitorConnection();
  void colorConnectCb();
  void depthConnectCb();
  void irConnectCb();

  void getSerialCb(const std::shared_ptr<openni2_camera::srv::GetSerial::Request> request,
                   std::shared_ptr<openni2_camera::srv::GetSerial::Response> response);

  rcl_interfaces::msg::SetParametersResult paramCb(const std::vector<rclcpp::Parameter> parameters);

  void applyConfigToOpenNIDevice();

  void genVideoModeTableMap();
  bool lookupVideoMode(const std::string& mode, OpenNI2VideoMode& video_mode);

  sensor_msgs::msg::Image::ConstSharedPtr rawToFloatingPointConversion(sensor_msgs::msg::Image::ConstSharedPtr raw_image);

  void setIRVideoMode(const OpenNI2VideoMode& ir_video_mode);
  void setColorVideoMode(const OpenNI2VideoMode& color_video_mode);
  void setDepthVideoMode(const OpenNI2VideoMode& depth_video_mode);

  int extractBusID(const std::string& uri) const;
  bool isConnected() const;

  void forceSetExposure();

  std::shared_ptr<OpenNI2DeviceManager> device_manager_;
  std::shared_ptr<OpenNI2Device> device_;

  std::string device_id_;
  int bus_id_;

  /** \brief indicates if reconnect logic is enabled. */
  bool enable_reconnect_;

  /** \brief indicates if the serialnumber is used in the camera names.
   * Default is false. The name is based on the device and vendor name.
   * This produces non-unique camera names, if multiple camaras of the
   * same type are used on the same machine.
   * Set to true, to get names with serialnumber. The names will always
   * be unique. This matches the 'openni' behaviour.
   */
  bool serialnumber_as_name_;

  /** \brief get_serial server*/
  rclcpp::Service<openni2_camera::srv::GetSerial>::SharedPtr get_serial_server;

  std::mutex connect_mutex_;
  // published topics
  image_transport::CameraPublisher pub_color_;
  image_transport::CameraPublisher pub_depth_;
  image_transport::CameraPublisher pub_depth_raw_;
  image_transport::CameraPublisher pub_ir_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_projector_info_;

  /** \brief timer for connection monitoring thread */
  rclcpp::TimerBase::SharedPtr timer_;

  /** \brief Camera info manager objects. */
  std::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_, ir_info_manager_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  OpenNI2VideoMode ir_video_mode_;
  OpenNI2VideoMode color_video_mode_;
  OpenNI2VideoMode depth_video_mode_;

  std::string ir_frame_id_;
  std::string color_frame_id_;
  std::string depth_frame_id_ ;

  std::string color_info_url_, ir_info_url_;

  bool color_depth_synchronization_;
  bool depth_registration_;

  std::map<std::string, OpenNI2VideoMode> video_modes_lookup_;

  // dynamic reconfigure config
  double depth_ir_offset_x_;
  double depth_ir_offset_y_;
  int z_offset_mm_;
  double z_scaling_;

  double ir_time_offset_;
  double color_time_offset_;
  double depth_time_offset_;

  int data_skip_;

  int data_skip_ir_counter_;
  int data_skip_color_counter_;
  int data_skip_depth_counter_;

  bool auto_exposure_;
  bool auto_white_balance_;
  int exposure_;

  bool ir_subscribers_;
  bool color_subscribers_;
  bool depth_subscribers_;
  bool depth_raw_subscribers_;
  bool projector_info_subscribers_;

  bool use_device_time_;
};

}

#endif
