/*
 * driver.h
 *
 *  Created on: Apr 8, 2013
 *      Author: jkammerl
 */

#include "OpenNI.h"

#include "openni2_camera/openni2_device.h"
#include "openni2_camera/openni2_exception.h"
#include "openni2_camera/openni2_convert.h"
#include "openni2_camera/openni2_frame_listener.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string>

namespace openni2_wrapper
{


OpenNI2Device::OpenNI2Device(const std::string& device_URI) throw (OpenNI2Exception)
{
  openni_device_ = boost::make_shared < openni::Device > ();

  openni::Status rc;
  if (device_URI.length()>0)
  {
    rc = openni_device_->open(device_URI.c_str());
  } else
  {
    rc = openni_device_->open(openni::ANY_DEVICE);
  }


  if (rc != openni::STATUS_OK)
    THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

  device_info_ = boost::make_shared<openni::DeviceInfo>();
  *device_info_ = openni_device_->getDeviceInfo();

  ir_frame_listener    = boost::make_shared<OpenNI2FrameListener>();
  color_frame_listener = boost::make_shared<OpenNI2FrameListener>();
  depth_frame_listener = boost::make_shared<OpenNI2FrameListener>();

}

OpenNI2Device::~OpenNI2Device()
{
  stopAllStreams();

  openni_device_->close();
}

const std::string OpenNI2Device::getUri() const
{
  return std::string(device_info_->getUri());
}

const std::string OpenNI2Device::getVendor() const
{
  return std::string(device_info_->getVendor());
}

const std::string OpenNI2Device::getName() const
{
  return std::string(device_info_->getName());
}

uint16_t OpenNI2Device::getUsbVendorId() const
{
  return device_info_->getUsbVendorId();
}

uint16_t OpenNI2Device::getUsbProductId() const
{
  return device_info_->getUsbProductId();
}

bool OpenNI2Device::hasIRSensor() const
{
  return openni_device_->hasSensor(openni::SENSOR_IR);
}

bool OpenNI2Device::hasColorSensor() const
{
  return openni_device_->hasSensor(openni::SENSOR_COLOR);
}

bool OpenNI2Device::hasDepthSensor() const
{
  return openni_device_->hasSensor(openni::SENSOR_DEPTH);
}

void OpenNI2Device::startIRStream()
{
  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    stream->start();
    stream->addNewFrameListener(ir_frame_listener.get());
  }

}


void OpenNI2Device::startColorStream()
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    stream->start();
    stream->addNewFrameListener(color_frame_listener.get());
  }
}
void OpenNI2Device::startDepthStream()
{
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    stream->start();
    stream->addNewFrameListener(depth_frame_listener.get());
  }
}

void OpenNI2Device::stopAllStreams()
{
  stopIRStream();
  stopColorStream();
  stopDepthStream();
}

void OpenNI2Device::stopIRStream()
{
  if (ir_video_stream_.get()!=0)
  {
    ir_video_stream_->removeNewFrameListener(ir_frame_listener.get());

    ir_video_stream_->stop();
    ir_video_stream_->destroy();

    ir_video_stream_.reset();
  }
}
void OpenNI2Device::stopColorStream()
{
  if (color_video_stream_.get()!=0)
  {
    color_video_stream_->removeNewFrameListener(color_frame_listener.get());

    color_video_stream_->stop();
    color_video_stream_->destroy();

    color_video_stream_.reset();
  }
}
void OpenNI2Device::stopDepthStream()
{
  if (depth_video_stream_.get()!=0)
  {
    depth_video_stream_->removeNewFrameListener(depth_frame_listener.get());

    depth_video_stream_->stop();
    depth_video_stream_->destroy();

    depth_video_stream_.reset();
  }
}

const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedIRVideoModes() const
{
  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  ir_video_modes_.clear();

  if (stream)
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo();

    ir_video_modes_ = openni2_convert(sensor_info.getSupportedVideoModes());
  }

  return ir_video_modes_;
}

const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedColorVideoModes() const
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  color_video_modes_.clear();

  if (stream)
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo();

    color_video_modes_ = openni2_convert(sensor_info.getSupportedVideoModes());
  }

  return color_video_modes_;
}

const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedDepthVideoModes() const
{
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  depth_video_modes_.clear();

  if (stream)
  {
    const openni::SensorInfo& sensor_info = stream->getSensorInfo();

    depth_video_modes_ = openni2_convert(sensor_info.getSupportedVideoModes());
  }

  return depth_video_modes_;
}

bool OpenNI2Device::isImageRegistrationModeSupported() const
{
  return openni_device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
}

void OpenNI2Device::setImageRegistrationMode(bool enabled) throw (OpenNI2Exception)
{
  openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  if (rc != openni::STATUS_OK)
    THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError());
}

void OpenNI2Device::setDepthColorSync(bool enabled) throw (OpenNI2Exception)
{
  openni::Status rc = openni_device_->setDepthColorSyncEnabled(enabled);
  if (rc != openni::STATUS_OK)
    THROW_OPENNI_EXCEPTION("Enabling depth color synchronization failed: \n%s\n", openni::OpenNI::getExtendedError());
}

const OpenNI2VideoMode OpenNI2Device::getIRVideoMode()
{
  OpenNI2VideoMode ret;

  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    openni::VideoMode video_mode = stream->getVideoMode();

    ret = openni2_convert(video_mode);
  } else
    THROW_OPENNI_EXCEPTION("Could not create video stream.");

  return ret;
}

const OpenNI2VideoMode OpenNI2Device::getColorVideoMode()
{
  OpenNI2VideoMode ret;

  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    openni::VideoMode video_mode = stream->getVideoMode();

    ret = openni2_convert(video_mode);
  } else
    THROW_OPENNI_EXCEPTION("Could not create video stream.");

  return ret;
}

const OpenNI2VideoMode OpenNI2Device::getDepthVideoMode()
{
  OpenNI2VideoMode ret;

  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    openni::VideoMode video_mode = stream->getVideoMode();

    ret = openni2_convert(video_mode);
  } else
    THROW_OPENNI_EXCEPTION("Could not create video stream.");

  return ret;
}

void OpenNI2Device::setIRVideoMode(const OpenNI2VideoMode& video_mode)
{
  boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

  if (stream)
  {
    const openni::VideoMode videoMode = openni2_convert(video_mode);
    const openni::Status rc =  stream->setVideoMode(videoMode);
    if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Couldn't IR video mode: \n%s\n", openni::OpenNI::getExtendedError());
  }
}

void OpenNI2Device::setColorVideoMode(const OpenNI2VideoMode& video_mode)
{
  boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

  if (stream)
  {
    const openni::VideoMode videoMode = openni2_convert(video_mode);
    const openni::Status rc =  stream->setVideoMode(videoMode);
    if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Couldn't color video mode: \n%s\n", openni::OpenNI::getExtendedError());
  }
}

void OpenNI2Device::setDepthVideoMode(const OpenNI2VideoMode& video_mode)
{
  boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

  if (stream)
  {
    const openni::VideoMode videoMode = openni2_convert(video_mode);
    const openni::Status rc =  stream->setVideoMode(videoMode);
    if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Couldn't depth video mode: \n%s\n", openni::OpenNI::getExtendedError());
  }
}

void OpenNI2Device::setIRFrameCallback(FrameCallbackFunction callback)
{
  ir_frame_listener->setCallback(callback);
}

void OpenNI2Device::setColorFrameCallback(FrameCallbackFunction callback)
{
  color_frame_listener->setCallback(callback);
}

void OpenNI2Device::setDepthFrameCallback(FrameCallbackFunction callback)
{
  depth_frame_listener->setCallback(callback);
}

boost::shared_ptr<openni::VideoStream> OpenNI2Device::getIRVideoStream() const
{
  if (ir_video_stream_.get() == 0)
  {
    if (hasIRSensor())
    {
      ir_video_stream_ = boost::make_shared<openni::VideoStream>();

      const openni::Status rc = ir_video_stream_->create(*openni_device_, openni::SENSOR_IR);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't create IR video stream: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  return ir_video_stream_;
}

boost::shared_ptr<openni::VideoStream> OpenNI2Device::getColorVideoStream() const
{
  if (color_video_stream_.get() == 0)
  {
    if (hasColorSensor())
    {
      color_video_stream_ = boost::make_shared<openni::VideoStream>();

      const openni::Status rc = color_video_stream_->create(*openni_device_, openni::SENSOR_COLOR);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't create color video stream: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  return color_video_stream_;
}

boost::shared_ptr<openni::VideoStream> OpenNI2Device::getDepthVideoStream() const
{
  if (depth_video_stream_.get() == 0)
  {
    if (hasDepthSensor())
    {
      depth_video_stream_ = boost::make_shared<openni::VideoStream>();

      const openni::Status rc = depth_video_stream_->create(*openni_device_, openni::SENSOR_DEPTH);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't create depth video stream: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  return depth_video_stream_;
}

std::ostream& operator << (std::ostream& stream, const OpenNI2Device& device) {

  stream << "Device info (" << device.getUri() << ")" << std::endl;
  stream << "   Vendor: " << device.getVendor() << std::endl;
  stream << "   Name: " << device.getName() << std::endl;
  stream << "   USB Vendor ID: " << device.getUsbVendorId() << std::endl;
  stream << "   USB Product ID: " << device.getUsbVendorId() << std::endl << std::endl;

  if (device.hasIRSensor())
  {
    stream << "IR sensor video modes:" << std::endl;
    const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedIRVideoModes();

    std::vector<OpenNI2VideoMode>::const_iterator it =  video_modes.begin();
    std::vector<OpenNI2VideoMode>::const_iterator it_end =  video_modes.end();
    for (;it !=it_end; ++it)
      stream << "   - " << *it << std::endl;
  } else
  {
    stream << "No IR sensor available" << std::endl;
  }

  if (device.hasColorSensor())
  {
    stream << "Color sensor video modes:" << std::endl;
    const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedColorVideoModes();

    std::vector<OpenNI2VideoMode>::const_iterator it =  video_modes.begin();
    std::vector<OpenNI2VideoMode>::const_iterator it_end =  video_modes.end();
    for (;it !=it_end; ++it)
      stream << "   - " << *it << std::endl;
  } else
  {
    stream << "No Color sensor available" << std::endl;
  }

  if (device.hasDepthSensor())
  {
    stream << "Depth sensor video modes:" << std::endl;
    const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedDepthVideoModes();

    std::vector<OpenNI2VideoMode>::const_iterator it =  video_modes.begin();
    std::vector<OpenNI2VideoMode>::const_iterator it_end =  video_modes.end();
    for (;it !=it_end; ++it)
      stream << "   - " << *it << std::endl;
  } else
  {
    stream << "No Depth sensor available" << std::endl;
  }

}


}
