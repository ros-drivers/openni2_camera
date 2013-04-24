/*
 * driver.h
 *
 *  Created on: Apr 8, 2013
 *      Author: jkammerl
 */

#ifndef OPENNI2_FRAME_LISTENER_H_
#define OPENNI2_FRAME_LISTENER_H_

#include "openni2_camera/openni2_device.h"

#include <sensor_msgs/Image.h>

#include <vector>

#include "OpenNI.h"

namespace openni2_wrapper
{

class OpenNI2FrameListener : public openni::VideoStream::NewFrameListener
{
public:
  OpenNI2FrameListener();

  virtual ~OpenNI2FrameListener()
  { };

  void onNewFrame(openni::VideoStream& stream);

  void setCallback(FrameCallbackFunction& callback)
  {
    callback_ = callback;
  }

private:
  openni::VideoFrameRef m_frame;

  FrameCallbackFunction callback_;
};

}

#endif
