/*
 * driver.h
 *
 *  Created on: Apr 8, 2013
 *      Author: jkammerl
 */

#ifndef OPENNI2_DEVICE_H
#define OPENNI2_DEVICE_H

#include "openni2_camera/openni2_video_mode.h"

#include "openni2_camera/openni2_exception.h"

#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

namespace openni
{
class Device;
class DeviceInfo;
class VideoStream;
class SensorInfo;
}

namespace openni2_wrapper
{

typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;

class OpenNI2FrameListener;

class OpenNI2Device
{
public:
  OpenNI2Device(const std::string& device_URI) throw (OpenNI2Exception);
  virtual ~OpenNI2Device();

  const std::string getUri() const;
  const std::string getVendor() const;
  const std::string getName() const;
  uint16_t getUsbVendorId() const;
  uint16_t getUsbProductId() const;

  bool hasIRSensor() const;
  bool hasColorSensor() const;
  bool hasDepthSensor() const;

  void startIRStream();
  void startColorStream();
  void startDepthStream();

  void stopAllStreams();

  void stopIRStream();
  void stopColorStream();
  void stopDepthStream();

  bool isImageRegistrationModeSupported() const;
  void setImageRegistrationMode(bool enabled) throw (OpenNI2Exception);
  void setDepthColorSync(bool enabled) throw (OpenNI2Exception);

  const OpenNI2VideoMode getIRVideoMode();
  const OpenNI2VideoMode getColorVideoMode();
  const OpenNI2VideoMode getDepthVideoMode();

  const std::vector<OpenNI2VideoMode>& getSupportedIRVideoModes() const;
  const std::vector<OpenNI2VideoMode>& getSupportedColorVideoModes() const;
  const std::vector<OpenNI2VideoMode>& getSupportedDepthVideoModes() const;

  void setIRVideoMode(const OpenNI2VideoMode& video_mode);
  void setColorVideoMode(const OpenNI2VideoMode& video_mode);
  void setDepthVideoMode(const OpenNI2VideoMode& video_mode);

  void setIRFrameCallback(FrameCallbackFunction callback);
  void setColorFrameCallback(FrameCallbackFunction callback);
  void setDepthFrameCallback(FrameCallbackFunction callback);

protected:

  boost::shared_ptr<openni::VideoStream> getIRVideoStream() const;
  boost::shared_ptr<openni::VideoStream> getColorVideoStream() const;
  boost::shared_ptr<openni::VideoStream> getDepthVideoStream() const;

  boost::shared_ptr<openni::Device> openni_device_;
  boost::shared_ptr<openni::DeviceInfo> device_info_;

  boost::shared_ptr<OpenNI2FrameListener> ir_frame_listener;
  boost::shared_ptr<OpenNI2FrameListener> color_frame_listener;
  boost::shared_ptr<OpenNI2FrameListener> depth_frame_listener;

  mutable boost::shared_ptr<openni::VideoStream> ir_video_stream_;
  mutable boost::shared_ptr<openni::VideoStream> color_video_stream_;
  mutable boost::shared_ptr<openni::VideoStream> depth_video_stream_;

  mutable std::vector<OpenNI2VideoMode> ir_video_modes_;
  mutable std::vector<OpenNI2VideoMode> color_video_modes_;
  mutable std::vector<OpenNI2VideoMode> depth_video_modes_;
};

std::ostream& operator << (std::ostream& stream, const OpenNI2Device& device);

}

#endif /* OPENNI_DEVICE_H */
