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

// PAL headers
#include <pal_vision_msgs/Gesture.h>
#include <pal_detection_msgs/PersonDetections.h>

// ROS headers
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// Boost headers
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

namespace openni2_wrapper
{ 

OpenNI2Driver::OpenNI2Driver(ros::NodeHandle& n, ros::NodeHandle& pnh) :
    nh_(n),
    pnh_(pnh),
    device_manager_(OpenNI2DeviceManager::getSingelton()),
    config_init_(false),
    data_skip_ir_counter_(0),
    data_skip_color_counter_(0),
    data_skip_depth_counter_ (0),
    ir_subscribers_(false),
    color_subscribers_(false),
    depth_subscribers_(false),
    depth_raw_subscribers_(false),
    gestures_subscribers_(false),
    num_users_subscribers_(false),
    user_map_subscribers_(false),
    user_tracker_image_transport_(nh_),
    next_available_color_id_(0)
{

  std::string cwd = boost::filesystem::current_path().string();
  std::string ini_file_path = ros::package::getPath("openni2_camera") + "/etc/NiTE.ini";
  std::string target_ini_file = cwd + "/NiTE.ini";
  if (!boost::filesystem::exists(target_ini_file))
  {
    ROS_WARN_STREAM("Making symlink from " << ini_file_path << " to " << target_ini_file << " This is required by NiTE2");
    boost::filesystem::create_symlink(ini_file_path, cwd + "/NiTE.ini");
  }

  genVideoModeTableMap();

  readConfigFromParameterServer();

  initDevice();

  // Initialize dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(pnh_));
  reconfigure_server_->setCallback(boost::bind(&OpenNI2Driver::configCb, this, _1, _2));

  while (!config_init_)
  {
    ROS_DEBUG("Waiting for dynamic reconfigure configuration.");
    boost::this_thread::sleep(boost::posix_time::seconds(0.1));
  }
  ROS_DEBUG("Dynamic reconfigure configuration received.");

  initializeUserColors();

  advertiseROSTopics(); 
}

void OpenNI2Driver::advertiseROSTopics()
{

  // Allow remapping namespaces rgb, ir, depth, depth_registered
  ros::NodeHandle color_nh(nh_, "rgb");
  image_transport::ImageTransport color_it(color_nh);
  ros::NodeHandle ir_nh(nh_, "ir");
  image_transport::ImageTransport ir_it(ir_nh);
  ros::NodeHandle depth_nh(nh_, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  ros::NodeHandle depth_raw_nh(nh_, "depth");
  image_transport::ImageTransport depth_raw_it(depth_raw_nh);
  // Advertise all published topics

  // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
  // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
  // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
  // the depth generator.
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  // Asus Xtion PRO does not have an RGB camera
  if (device_->hasColorSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&OpenNI2Driver::colorConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::colorConnectCb, this);
    pub_color_ = color_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  }

  if (device_->hasIRSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&OpenNI2Driver::irConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::irConnectCb, this);
    pub_ir_ = ir_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  }

  if (device_->hasDepthSensor())
  {
    image_transport::SubscriberStatusCallback itssc = boost::bind(&OpenNI2Driver::depthConnectCb, this);
    ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::depthConnectCb, this);
    pub_depth_raw_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    pub_depth_ = depth_raw_it.advertiseCamera("image", 1, itssc, itssc, rssc, rssc);
  }

  if ( device_->hasHandTracker() )
  {
    ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::handTrackerConnectCb, this);
    pub_gestures_ = nh_.advertise<pal_vision_msgs::Gesture>("gestures", 1, rssc, rssc);
  }

  if ( device_->hasUserTracker() )
  {
    ros::SubscriberStatusCallback rssc = boost::bind(&OpenNI2Driver::userTrackerConnectCb, this);
    pub_users_ = nh_.advertise<pal_detection_msgs::PersonDetections>("users", 1, rssc, rssc);
    image_transport::SubscriberStatusCallback itssc = boost::bind(&OpenNI2Driver::userTrackerConnectCb, this);
    pub_user_map_ = user_tracker_image_transport_.advertise("user_map", 1, itssc, itssc);

    geometry_msgs::TransformStamped cameraPose;
    //request camera pose to see if it is available in TF
    publish_camera_pose_ = getCameraPose(cameraPose);
    if ( !publish_camera_pose_ )
      ROS_INFO("The camera pose won't be published as it is not in TF");

    device_->setUserTrackerFrameCallback(boost::bind(&OpenNI2Driver::newUserTrackerFrameCallback, this, _1, _2));

    ROS_INFO("Starting user tracker.");
    device_->startUserTracker();
  }

  ////////// CAMERA INFO MANAGER

  // Pixel offset between depth and IR images.
  // By default assume offset of (5,4) from 9x7 correlation window.
  // NOTE: These are now (temporarily?) dynamically reconfigurable, to allow tweaking.
  //param_nh.param("depth_ir_offset_x", depth_ir_offset_x_, 5.0);
  //param_nh.param("depth_ir_offset_y", depth_ir_offset_y_, 4.0);

  // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
  // camera_info_manager substitutes this for ${NAME} in the URL.
  std::string serial_number = device_->getStringID();
  std::string color_name, ir_name;

  color_name = "rgb_"   + serial_number;
  ir_name  = "depth_" + serial_number;

  // Load the saved calibrations, if they exist
  color_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(color_nh, color_name, color_info_url_);
  ir_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(ir_nh,  ir_name,  ir_info_url_);

}

void OpenNI2Driver::configCb(Config &config, uint32_t level)
{
  bool stream_reset = false;

  depth_ir_offset_x_ = config.depth_ir_offset_x;
  depth_ir_offset_y_ = config.depth_ir_offset_y;
  z_offset_mm_ = config.z_offset_mm;
  z_scaling_ = config.z_scaling;

  ir_time_offset_ = ros::Duration(config.ir_time_offset);
  color_time_offset_ = ros::Duration(config.color_time_offset);
  depth_time_offset_ = ros::Duration(config.depth_time_offset);

  if (lookupVideoModeFromDynConfig(config.ir_mode, ir_video_mode_)<0)
  {
    ROS_ERROR("Undefined IR video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config.color_mode, color_video_mode_)<0)
  {
    ROS_ERROR("Undefined color video mode received from dynamic reconfigure");
    exit(-1);
  }

  if (lookupVideoModeFromDynConfig(config.depth_mode, depth_video_mode_)<0)
  {
    ROS_ERROR("Undefined depth video mode received from dynamic reconfigure");
    exit(-1);
  }

  // assign pixel format

  ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
  color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
  depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

  color_depth_synchronization_ = config.color_depth_synchronization;
  depth_registration_ = config.depth_registration;

  auto_exposure_ = config.auto_exposure;
  auto_white_balance_ = config.auto_white_balance;

  use_device_time_ = config.use_device_time;

  data_skip_ = config.data_skip+1;

  applyConfigToOpenNIDevice();

  config_init_ = true;

  old_config_ = config;
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
    ROS_ERROR_STREAM("Unsupported IR video mode - " << ir_video_mode);
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
    ROS_ERROR_STREAM("Unsupported color video mode - " << color_video_mode);
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
    ROS_ERROR_STREAM("Unsupported depth video mode - " << depth_video_mode);
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
      if (!config_init_ || (old_config_.depth_registration != depth_registration_))
        device_->setImageRegistrationMode(depth_registration_);
    }
    catch (const OpenNI2Exception& exception)
    {
      ROS_ERROR("Could not set image registration. Reason: %s", exception.what());
    }
  }

  try
  {
    if (!config_init_ || (old_config_.color_depth_synchronization != color_depth_synchronization_))
      device_->setDepthColorSync(color_depth_synchronization_);
  }
  catch (const OpenNI2Exception& exception)
  {
    ROS_ERROR("Could not set color depth synchronization. Reason: %s", exception.what());
  }

  try
  {
    if (!config_init_ || (old_config_.auto_exposure != auto_exposure_))
      device_->setAutoExposure(auto_exposure_);
  }
  catch (const OpenNI2Exception& exception)
  {
    ROS_ERROR("Could not set auto exposure. Reason: %s", exception.what());
  }

  try
  {
    if (!config_init_ || (old_config_.auto_white_balance != auto_white_balance_))
      device_->setAutoWhiteBalance(auto_white_balance_);
  }
  catch (const OpenNI2Exception& exception)
  {
    ROS_ERROR("Could not set auto white balance. Reason: %s", exception.what());
  }

  device_->setUseDeviceTimer(use_device_time_);

}

void OpenNI2Driver::colorConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  color_subscribers_ = pub_color_.getNumSubscribers() > 0;

  if (color_subscribers_ && !device_->isColorStreamStarted())
  {
    // Can't stream IR and RGB at the same time. Give RGB preference.
    if (device_->isIRStreamStarted())
    {
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
      ROS_INFO("Stopping IR stream.");
      device_->stopIRStream();
    }

    device_->setColorFrameCallback(boost::bind(&OpenNI2Driver::newColorFrameCallback, this, _1));

    ROS_INFO("Starting color stream.");
    device_->startColorStream();

  }
  else if (!color_subscribers_ && device_->isColorStreamStarted())
  {
    ROS_INFO("Stopping color stream.");
    device_->stopColorStream();

    // Start IR if it's been blocked on RGB subscribers
    bool need_ir = pub_ir_.getNumSubscribers() > 0;
    if (need_ir && !device_->isIRStreamStarted())
    {
      device_->setIRFrameCallback(boost::bind(&OpenNI2Driver::newIRFrameCallback, this, _1));

      ROS_INFO("Starting IR stream.");
      device_->startIRStream();
    }
  }
}

std::string getGestureName( nite::GestureType type )
{
  if ( type == nite::GESTURE_WAVE )
    return "Wave";

  if ( type == nite::GESTURE_CLICK )
    return "Click";

  return "unknown";
}

void OpenNI2Driver::depthConnectCb()
{  
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  depth_subscribers_ = pub_depth_.getNumSubscribers() > 0;
  depth_raw_subscribers_ = pub_depth_raw_.getNumSubscribers() > 0;

  bool need_depth = depth_subscribers_ || depth_raw_subscribers_;

  if (need_depth && !device_->isDepthStreamStarted())
  {
    device_->setDepthFrameCallback(boost::bind(&OpenNI2Driver::newDepthFrameCallback, this, _1));

    ROS_INFO("Starting depth stream.");
    device_->startDepthStream();
  }
  else if (!need_depth && device_->isDepthStreamStarted())
  {
    ROS_INFO("Stopping depth stream.");
    device_->stopDepthStream();
  }
}

void OpenNI2Driver::irConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  ir_subscribers_ = pub_ir_.getNumSubscribers() > 0;

  if (ir_subscribers_ && !device_->isIRStreamStarted())
  {
    // Can't stream IR and RGB at the same time
    if (device_->isColorStreamStarted())
    {
      ROS_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
    }
    else
    {
      device_->setIRFrameCallback(boost::bind(&OpenNI2Driver::newIRFrameCallback, this, _1));

      ROS_INFO("Starting IR stream.");
      device_->startIRStream();
    }
  }
  else if (!ir_subscribers_ && device_->isIRStreamStarted())
  {
    ROS_INFO("Stopping IR stream.");
    device_->stopIRStream();
  }
}

void OpenNI2Driver::handTrackerConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  gestures_subscribers_ = pub_gestures_.getNumSubscribers() > 0;

  bool need_hand_tracker = gestures_subscribers_;

  if (need_hand_tracker && !device_->isHandTrackerStarted())
  {
    device_->setHandTrackerFrameCallback(boost::bind(&OpenNI2Driver::newHandTrackerFrameCallback, this, _1));

    ROS_INFO("Starting hand tracker.");
    device_->startHandTracker();
  }
  else if (!need_hand_tracker && device_->isHandTrackerStarted())
  {
    ROS_INFO("Stopping hand tracker.");
    device_->stopHandTracker();
  }
}

void OpenNI2Driver::userTrackerConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  num_users_subscribers_ = pub_users_.getNumSubscribers() > 0;
  user_map_subscribers_  = pub_user_map_.getNumSubscribers() > 0;
}

void OpenNI2Driver::newIRFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_ir_counter_)%data_skip_==0)
  {
    data_skip_ir_counter_ = 0;

    if (ir_subscribers_)
    {
      image->header.frame_id = ir_frame_id_;
      image->header.stamp = image->header.stamp + ir_time_offset_;

      pub_ir_.publish(image, getIRCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void OpenNI2Driver::newColorFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_color_counter_)%data_skip_==0)
  {
    data_skip_color_counter_ = 0;

    if (color_subscribers_)
    {
      image->header.frame_id = color_frame_id_;
      image->header.stamp = image->header.stamp + color_time_offset_;

      pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
    }
  }
}

void OpenNI2Driver::newDepthFrameCallback(sensor_msgs::ImagePtr image)
{
  if ((++data_skip_depth_counter_)%data_skip_==0)
  {

    data_skip_depth_counter_ = 0;

    if (depth_raw_subscribers_||depth_subscribers_)
    {
      image->header.stamp = image->header.stamp + depth_time_offset_;

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

      sensor_msgs::CameraInfoPtr cam_info;

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
        sensor_msgs::ImageConstPtr floating_point_image = rawToFloatingPointConversion(image);
        pub_depth_.publish(floating_point_image, cam_info);
      }
    }
  }
}

void OpenNI2Driver::newHandTrackerFrameCallback(nite::HandTrackerFrameRef handTrackerFrame)
{
  const nite::Array<nite::GestureData>& gestures = handTrackerFrame.getGestures();
  ros::Time now = ros::Time::now();
  for (int i = 0; i < gestures.getSize(); ++i)
  {
    if ( gestures[i].isComplete() )
    {
      pal_vision_msgs::Gesture msg;
      msg.header.stamp = now;
      msg.header.frame_id = depth_frame_id_;
      msg.gestureId = getGestureName(gestures[i].getType());
      msg.position3D.x   = gestures[i].getCurrentPosition().x / 1000;
      msg.position3D.y   = gestures[i].getCurrentPosition().y / 1000;
      msg.position3D.z   = gestures[i].getCurrentPosition().z / 1000;

      ROS_INFO_STREAM("Gesture: " << msg.gestureId <<
                      " at point (" << msg.position3D.x << ", " <<
                      msg.position3D.y << ", " << msg.position3D.z << ")");

      pub_gestures_.publish(msg);
    }
  }
}

bool OpenNI2Driver::getCameraPose(geometry_msgs::TransformStamped& cameraPose)
{  
  //Get the camera frame pose wrt /base_link
  std::string referenceFrame = "/base_link";
  double timeOutMs = 1000;
  std::string errMsg;
  if ( !tf_listener_.waitForTransform(referenceFrame, color_frame_id_,
                                  ros::Time(0),                    //get the most up to date transformation
                                  ros::Duration(timeOutMs/1000.0), //time out
                                  ros::Duration(0.01),             //checking rate
                                  &errMsg) )                       //error message in case of failure
  {
    ROS_ERROR_STREAM("Unable to get TF transform from " << color_frame_id_ << " to " <<
                     referenceFrame << ": " << errMsg);
    return false;
  }

  try
  {
    tf::StampedTransform tfCameraPose;
    tf_listener_.lookupTransform( referenceFrame, color_frame_id_,   //get transform from ... to ...
                                  ros::Time(0),                      //get latest available
                                  tfCameraPose);

    tf::transformStampedTFToMsg(tfCameraPose, cameraPose);
  }
  catch ( const tf::TransformException& e)
  {
    ROS_ERROR_STREAM("Error in lookUpTransform from " << color_frame_id_ << " to " <<
                     referenceFrame);
    return false;
  }

  return true;
}

void OpenNI2Driver::publishUsers(nite::UserTrackerFrameRef userTrackerFrame)
{
  pal_detection_msgs::PersonDetections detectionsMsg;

  if ( publish_camera_pose_ )
    getCameraPose(detectionsMsg.camera_pose);

  detectionsMsg.header.stamp = ros::Time::now();

  const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
  for (int i = 0; i < users.getSize(); ++i)
  {
    const nite::UserData& user = users[i];
    if ( user.getSkeleton().getState() == nite::SKELETON_TRACKED &&
         user.isVisible() && !user.isLost() )
    {
      pal_detection_msgs::PersonDetection detectionMsg;


      std::cout << "Bounding box  min = (" << user.getBoundingBox().min.x << ", " << user.getBoundingBox().min.y << ", " << user.getBoundingBox().min.z << ")" <<
                   " max = (" << user.getBoundingBox().max.x << ", " << user.getBoundingBox().max.y << ", " << user.getBoundingBox().max.z << ")" << std::endl;
      int depthX = user.getBoundingBox().min.x + (user.getBoundingBox().max.x - user.getBoundingBox().min.x)/2;
      int depthY = user.getBoundingBox().min.y + (user.getBoundingBox().max.y - user.getBoundingBox().min.y)/2;
      float worldX;
      float worldY;
      float worldZ;

      openni::DepthPixel *depth_pixel = (openni::DepthPixel *)userTrackerFrame.getDepthFrame().getData();
      openni::DepthPixel pixel = depth_pixel[depthY * userTrackerFrame.getDepthFrame().getVideoMode().getResolutionX() + depthX];

      openni::CoordinateConverter::convertDepthToWorld(*(device_->getDepthVideoStream()),
                                                       depthX,
                                                       depthY,
                                                       pixel,
                                                       &worldX,
                                                       &worldY,
                                                       &worldZ);

      detectionMsg.position3D.header.frame_id = depth_frame_id_;
      detectionMsg.position3D.point.x = user.getCenterOfMass().x/1000.0; //from mm to m
      detectionMsg.position3D.point.y = user.getCenterOfMass().y/1000.0;
      detectionMsg.position3D.point.z = user.getCenterOfMass().z/1000.0;

      std::cout << "User position from depth image:        " << worldX/1000 << ", " << worldY/1000 << ", " << worldZ/1000 << " m " << std::endl;
      std::cout << "user position provided by UserTracker: " << detectionMsg.position3D.point.x << ", " << detectionMsg.position3D.point.y << ", " << detectionMsg.position3D.point.z << " m" << std::endl;
      std::cout << std::endl;
      std::stringstream ss;
      ss << user.getId();
      detectionMsg.face.name = ss.str();

      detectionsMsg.persons.push_back(detectionMsg);
    }    
  }

  if ( !detectionsMsg.persons.empty() )
    pub_users_.publish(detectionsMsg);
}

void OpenNI2Driver::drawSkeletonLink(nite::UserTracker& userTracker,
                                     const nite::SkeletonJoint& joint1,
                                     const nite::SkeletonJoint& joint2,
                                     cv::Mat& img)
{
  if ( joint1.getPositionConfidence() >= 0.5 &&
       joint2.getPositionConfidence() >= 0.5 )
  {
    float coordinates[6] = {0};
    userTracker.convertJointCoordinatesToDepth(joint1.getPosition().x,
                                               joint1.getPosition().y,
                                               joint1.getPosition().z,
                                               &coordinates[0], &coordinates[1]);

    userTracker.convertJointCoordinatesToDepth(joint2.getPosition().x,
                                               joint2.getPosition().y,
                                               joint2.getPosition().z,
                                               &coordinates[3], &coordinates[4]);

    cv::line(img,
             cv::Point(coordinates[0], coordinates[1]),
             cv::Point(coordinates[3], coordinates[4]),
             cv::Scalar(255,255,255), 2);
  }
}

void OpenNI2Driver::drawSkeleton(nite::UserTracker& userTracker,
                                 const nite::UserData& user,
                                 cv::Mat& img)
{
  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_HEAD),
                   user.getSkeleton().getJoint(nite::JOINT_NECK), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW),
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW),
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER),
                   user.getSkeleton().getJoint(nite::JOINT_TORSO), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),
                   user.getSkeleton().getJoint(nite::JOINT_TORSO), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_TORSO),
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_TORSO),
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE),
                   user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP),
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), img);

  drawSkeletonLink(userTracker,
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE),
                   user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), img);
}

void OpenNI2Driver::publishUserMap(nite::UserTrackerFrameRef userTrackerFrame,
                                   nite::UserTracker& userTracker)
{
  nite::UserMap userMap = userTrackerFrame.getUserMap();
  cv::Mat cvUserMap(userMap.getHeight(), userMap.getWidth(),
                    CV_16UC1, reinterpret_cast<void*>(const_cast<nite::UserId*>(userMap.getPixels())),
                    userMap.getStride());

  cv::Mat userImage = cv::Mat::zeros(userMap.getHeight(), userMap.getWidth(), CV_8UC3);

  const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
  for (int i = 0; i < users.getSize(); ++i)
  {
    const nite::UserData& user = users[i];
    nite::UserId nId = user.getId();

    if ( user.getSkeleton().getState() == nite::SKELETON_TRACKED &&
         user.isVisible() && !user.isLost() )
    {      
      cv::Mat userMask = (cvUserMap == nId);
      if ( user_id_color_.find(nId) == user_id_color_.end() )
      {
        user_id_color_[nId] = user_colors_available_[next_available_color_id_];
        ++next_available_color_id_;
        if ( next_available_color_id_ == user_colors_available_.size() )
          next_available_color_id_ = 0;
      }
      //paint user mask in the image
      userImage.setTo(user_id_color_[nId], userMask);

      drawSkeleton(userTracker, user, userImage);
    }
  }

  cv_image_.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_.image = userImage;
  sensor_msgs::Image img_msg;
  img_msg.header.stamp = ros::Time::now();
  cv_image_.toImageMsg(img_msg);
  pub_user_map_.publish(img_msg);
}

void publishTransform(const nite::UserData& user, nite::JointType const& joint_name, const std::string& frame_id,
                      const std::string& child_frame_id) {
    static tf::TransformBroadcaster br;
    const nite::SkeletonJoint joint = user.getSkeleton().getJoint(joint_name);
    const nite::Point3f joint_position = joint.getPosition();
    const nite::Quaternion joint_orientation = joint.getOrientation();

    double x = -joint_position.x / 1000.0;
    double y = joint_position.y / 1000.0;
    double z = joint_position.z / 1000.0;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));

    transform.setRotation(tf::Quaternion(joint_orientation.x, joint_orientation.y,
                                         joint_orientation.z, joint_orientation.w));
    if (isnan(transform.getRotation().x()) || isnan(transform.getRotation().y()) ||
        isnan(transform.getRotation().z()) || isnan(transform.getRotation().w()))
    {
      ROS_WARN_STREAM_ONCE_NAMED(std::string("publishTransform ") + child_frame_id, "Got nan on frame " << child_frame_id <<
                          " orientation. Publishing it with identity frame. Will only publish this message once per joint");
      transform.setRotation(tf::Quaternion::getIdentity());
    }

    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}

void publishTransforms(nite::UserTrackerFrameRef userTracker, const std::string& frame_id) {
  for (int i = 0; i < userTracker.getUsers().getSize(); ++i)
  {
    const nite::UserData& user = userTracker.getUsers()[i];
    if (user.getSkeleton().getState() != nite::SKELETON_TRACKED)
      continue;

        publishTransform(user, nite::JOINT_HEAD,           frame_id, "head");
        publishTransform(user, nite::JOINT_NECK,           frame_id, "neck");
        publishTransform(user, nite::JOINT_TORSO,          frame_id, "torso");

        publishTransform(user, nite::JOINT_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, nite::JOINT_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, nite::JOINT_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, nite::JOINT_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, nite::JOINT_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, nite::JOINT_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, nite::JOINT_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, nite::JOINT_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, nite::JOINT_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, nite::JOINT_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, nite::JOINT_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, nite::JOINT_RIGHT_FOOT,     frame_id, "right_foot");
    }
}

void OpenNI2Driver::newUserTrackerFrameCallback(nite::UserTrackerFrameRef userTrackerFrame,
                                                nite::UserTracker& userTracker)
{
  const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
  for (int i = 0; i < users.getSize(); ++i)
  {
    const nite::UserData& user = users[i];

    //try to start tracking the skeleton of new users
    if (user.isNew())
    {
      userTracker.startSkeletonTracking(user.getId());
    }
  }

  if (num_users_subscribers_)
  {
    //publish num of users
    publishUsers(userTrackerFrame);
  }

  if (user_map_subscribers_)
  {
    //publisher user's segmentation map
    publishUserMap(userTrackerFrame,
                   userTracker);
  }

  publishTransforms(userTrackerFrame, "/camera_link");
}

// Methods to get calibration parameters for the various cameras
sensor_msgs::CameraInfoPtr OpenNI2Driver::getDefaultCameraInfo(int width, int height, double f) const
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];     // cx
  info->P[6]  = info->K[5];     // cy
  info->P[10] = 1.0;

  return info;
}

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::CameraInfoPtr OpenNI2Driver::getColorCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (color_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(color_info_manager_->getCameraInfo());
    if ( static_cast<int>(info->width) != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the RGB camera. Using default parameters.");
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


sensor_msgs::CameraInfoPtr OpenNI2Driver::getIRCameraInfo(int width, int height, ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (ir_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_->getCameraInfo());
    if ( static_cast<int>(info->width) != width )
    {
      // Use uncalibrated values
      ROS_WARN_ONCE("Image resolution doesn't match calibration of the IR camera. Using default parameters.");
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

sensor_msgs::CameraInfoPtr OpenNI2Driver::getDepthCameraInfo(int width, int height, ros::Time time) const
{
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See http://www.ros.org/wiki/kinect_calibration/technical

  double scaling = (double)width / 640;

  sensor_msgs::CameraInfoPtr info = getIRCameraInfo(width, height, time);
  info->K[2] -= depth_ir_offset_x_*scaling; // cx
  info->K[5] -= depth_ir_offset_y_*scaling; // cy
  info->P[2] -= depth_ir_offset_x_*scaling; // cx
  info->P[6] -= depth_ir_offset_y_*scaling; // cy

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

void OpenNI2Driver::readConfigFromParameterServer()
{
  if (!pnh_.getParam("device_id", device_id_))
  {
    ROS_WARN ("~device_id is not set! Using first device.");
    device_id_ = "#1";
  }

  // Camera TF frames
  pnh_.param("ir_frame_id", ir_frame_id_, std::string("/openni_ir_optical_frame"));
  pnh_.param("rgb_frame_id", color_frame_id_, std::string("/openni_rgb_optical_frame"));
  pnh_.param("depth_frame_id", depth_frame_id_, std::string("/openni_depth_optical_frame"));

  ROS_DEBUG("ir_frame_id = '%s' ", ir_frame_id_.c_str());
  ROS_DEBUG("rgb_frame_id = '%s' ", color_frame_id_.c_str());
  ROS_DEBUG("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  pnh_.param("rgb_camera_info_url", color_info_url_, std::string());
  pnh_.param("depth_camera_info_url", ir_info_url_, std::string());

}

std::string OpenNI2Driver::resolveDeviceURI(const std::string& device_id) throw(OpenNI2Exception)
{
  std::string device_URI;
  boost::shared_ptr<std::vector<std::string> > available_device_URIs = 
    device_manager_->getConnectedDeviceURIs();

  // look for '#<number>' format
  if (device_id_.size() > 1 && device_id_[0] == '#')
  {
    std::istringstream device_number_str(device_id_.substr(1));
    int device_number;
    device_number_str >> device_number;
    int device_index = device_number - 1; // #1 refers to first device
    if (device_index >= static_cast<int>(available_device_URIs->size()) || device_index < 0)
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
  else if (device_id_.size() > 1 && device_id_.find('@') != std::string::npos)
  {
    // get index of @ character
    size_t index = device_id_.find('@');
    if (index == 0 || index == std::string::npos )
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give the bus number before the @.",
        device_id_.c_str());
    }
    if (index >= device_id_.size() - 1)
    {
      THROW_OPENNI_EXCEPTION(
        "%s is not a valid device URI, you must give a number after the @, specify 0 for first device",
        device_id_.c_str());
    }

    // pull out device number on bus
    std::istringstream device_number_str(device_id_.substr(index+1));
    int device_number;
    device_number_str >> device_number;

    // reorder to @<bus>
    std::string bus = device_id_.substr(0, index);
    bus.insert(0, "@");

    for (size_t i = 0; i < available_device_URIs->size(); ++i)
    {
      std::string s = (*available_device_URIs)[i];
      if (s.find(bus) != std::string::npos)
      {
        // this matches our bus, check device number
        --device_number;
        if (device_number <= 0)
          return s;
      }
    }

    THROW_OPENNI_EXCEPTION("Device not found %s", device_id_.c_str());
  }
  // everything else is treated as device_URI directly
  else
  {
    return device_id_;
  }

  return "";
}

void OpenNI2Driver::initDevice()
{
  while (ros::ok() && !device_)
  {
    try
    {
      std::string device_URI = resolveDeviceURI(device_id_);
      device_ = device_manager_->getDevice(device_URI);
    }
    catch (const OpenNI2Exception& exception)
    {
      if (!device_)
      {
        ROS_INFO("No matching device found.... waiting for devices. Reason: %s", exception.what());
        boost::this_thread::sleep(boost::posix_time::seconds(3));
        continue;
      }
      else
      {
        ROS_ERROR("Could not retrieve device. Reason: %s", exception.what());
        exit(-1);
      }
    }
  }

  while (ros::ok() && !device_->isValid())
  {
    ROS_DEBUG("Waiting for device initialization..");
    boost::this_thread::sleep(boost::posix_time::seconds(0.1));
  }

}

void OpenNI2Driver::genVideoModeTableMap()
{
  /*
   * #  Video modes defined by dynamic reconfigure:
output_mode_enum = gen.enum([  gen.const(  "SXGA_30Hz", int_t, 1,  "1280x1024@30Hz"),
                               gen.const(  "SXGA_15Hz", int_t, 2,  "1280x1024@15Hz"),
                               gen.const(   "XGA_30Hz", int_t, 3,  "1280x720@30Hz"),
                               gen.const(   "XGA_15Hz", int_t, 4,  "1280x720@15Hz"),
                               gen.const(   "VGA_30Hz", int_t, 5,  "640x480@30Hz"),
                               gen.const(   "VGA_25Hz", int_t, 6,  "640x480@25Hz"),
                               gen.const(  "QVGA_25Hz", int_t, 7,  "320x240@25Hz"),
                               gen.const(  "QVGA_30Hz", int_t, 8,  "320x240@30Hz"),
                               gen.const(  "QVGA_60Hz", int_t, 9,  "320x240@60Hz"),
                               gen.const( "QQVGA_25Hz", int_t, 10, "160x120@25Hz"),
                               gen.const( "QQVGA_30Hz", int_t, 11, "160x120@30Hz"),
                               gen.const( "QQVGA_60Hz", int_t, 12, "160x120@60Hz")],
                               "output mode")
  */

  video_modes_lookup_.clear();

  OpenNI2VideoMode video_mode;

  // SXGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[1] = video_mode;

  // SXGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 1024;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[2] = video_mode;

  // XGA_30Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[3] = video_mode;

  // XGA_15Hz
  video_mode.x_resolution_ = 1280;
  video_mode.y_resolution_ = 720;
  video_mode.frame_rate_ = 15;

  video_modes_lookup_[4] = video_mode;

  // VGA_30Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[5] = video_mode;

  // VGA_25Hz
  video_mode.x_resolution_ = 640;
  video_mode.y_resolution_ = 480;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[6] = video_mode;

  // QVGA_25Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[7] = video_mode;

  // QVGA_30Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[8] = video_mode;

  // QVGA_60Hz
  video_mode.x_resolution_ = 320;
  video_mode.y_resolution_ = 240;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[9] = video_mode;

  // QQVGA_25Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 25;

  video_modes_lookup_[10] = video_mode;

  // QQVGA_30Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 30;

  video_modes_lookup_[11] = video_mode;

  // QQVGA_60Hz
  video_mode.x_resolution_ = 160;
  video_mode.y_resolution_ = 120;
  video_mode.frame_rate_ = 60;

  video_modes_lookup_[12] = video_mode;

}

int OpenNI2Driver::lookupVideoModeFromDynConfig(int mode_nr, OpenNI2VideoMode& video_mode)
{
  int ret = -1;

  std::map<int, OpenNI2VideoMode>::const_iterator it;

  it = video_modes_lookup_.find(mode_nr);

  if (it!=video_modes_lookup_.end())
  {
    video_mode = it->second;
    ret = 0;
  }

  return ret;
}

sensor_msgs::ImageConstPtr OpenNI2Driver::rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image)
{
  static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  sensor_msgs::ImagePtr new_image = boost::make_shared<sensor_msgs::Image>();

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

void OpenNI2Driver::initializeUserColors()
{
  user_colors_available_.clear();
  user_colors_available_.push_back( cv::Scalar(255,0,0) );
  user_colors_available_.push_back( cv::Scalar(0,255,0) );
  user_colors_available_.push_back( cv::Scalar(0,0,255) );
  user_colors_available_.push_back( cv::Scalar(255,255,0) );
  user_colors_available_.push_back( cv::Scalar(255,0,255) );
  user_colors_available_.push_back( cv::Scalar(0,255,255) );
  user_colors_available_.push_back( cv::Scalar(128,128,128) );

  next_available_color_id_ = 0;
}

}
