
#include "OpenNI.h"

#include "openni2_camera/openni2_frame_listener.h"

#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>

namespace openni2_wrapper
{

OpenNI2FrameListener::OpenNI2FrameListener() :
    callback_(0),
    prev_device_time_(0)
{
  ros::Time::init();
}

void OpenNI2FrameListener::onNewFrame(openni::VideoStream& stream)
{
  stream.readFrame(&m_frame);

  if (m_frame.isValid() && callback_)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);

    image->header.stamp = ros::Time::now();

    uint64_t device_time = m_frame.getTimestamp();

    //uint64_t diff_time = device_time-prev_device_time_;

    prev_device_time_ = device_time;

    image->width = m_frame.getWidth();
    image->height = m_frame.getHeight();

    std::size_t data_size = m_frame.getDataSize();

    image->data.resize(data_size);
    memcpy(&image->data[0], m_frame.getData(), data_size);

    image->is_bigendian = 0;

    const openni::VideoMode& video_mode = m_frame.getVideoMode();
    switch (video_mode.getPixelFormat())
    {
      case openni::PIXEL_FORMAT_DEPTH_1_MM:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2;
        break;
      case openni::PIXEL_FORMAT_DEPTH_100_UM:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2;
        break;
      case openni::PIXEL_FORMAT_SHIFT_9_2:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2;
        break;
      case openni::PIXEL_FORMAT_SHIFT_9_3:
        image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        image->step = sizeof(unsigned char) * 2;
        break;

      case openni::PIXEL_FORMAT_RGB888:
        image->encoding = sensor_msgs::image_encodings::RGB8;
        image->step = sizeof(unsigned char) * 3;
        break;
      case openni::PIXEL_FORMAT_YUV422:
        image->encoding = sensor_msgs::image_encodings::YUV422;
        image->step = sizeof(unsigned char) * 4;
        break;
      case openni::PIXEL_FORMAT_GRAY8:
        image->encoding = sensor_msgs::image_encodings::MONO8;
        image->step = sizeof(unsigned char) * 1;
        break;
      case openni::PIXEL_FORMAT_GRAY16:
        image->encoding = sensor_msgs::image_encodings::MONO16;
        image->step = sizeof(unsigned char) * 2;
        break;
      case openni::PIXEL_FORMAT_JPEG:
      default:
        ROS_ERROR("Invalid image encoding");
        break;
    }

    callback_(image);
  }

}

}

