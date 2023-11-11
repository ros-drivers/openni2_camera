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

#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_device.h"

#include <iostream>

#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace openni2_wrapper;

int ir_counter_ = 0;
int color_counter_ = 0;
int depth_counter_ = 0;

void IRCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  ++ir_counter_;
}

void ColorCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  ++color_counter_;
}

void DepthCallback(sensor_msgs::msg::Image::SharedPtr image)
{
  ++depth_counter_;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("test_wrapper");

  OpenNI2DeviceManager device_manager;

  std::cout << device_manager;

  std::shared_ptr<std::vector<std::string> > device_uris = device_manager.getConnectedDeviceURIs();

  for (const std::string& uri : *device_uris)
  {
    std::shared_ptr<OpenNI2Device> device = device_manager.getDevice(uri, &node);

    std::cout << *device;

    device->setIRFrameCallback(std::bind(&IRCallback, std::placeholders::_1));
    device->setColorFrameCallback(std::bind(&ColorCallback, std::placeholders::_1));
    device->setDepthFrameCallback(std::bind(&DepthCallback, std::placeholders::_1));

    ir_counter_ = 0;
    color_counter_ = 0;
    depth_counter_ = 0;

    device->startColorStream();
    device->startDepthStream();

    std::this_thread::sleep_for(1000ms);

    device->stopAllStreams();

    std::cout<<std::endl;

    std::cout<<"Number of called to IRCallback: "<< ir_counter_ << std::endl;
    std::cout<<"Number of called to ColorCallback: "<< color_counter_ << std::endl;
    std::cout<<"Number of called to DepthCallback: "<< depth_counter_ << std::endl;
  }

  return 0;
}
