#include "openni2_camera/nite2_user_tracker_frame_listener.h"

#include "NiTE-2/NiTE.h"

#include <ros/ros.h>

namespace nite2_wrapper
{

NiTE2UserTrackerFrameListener::NiTE2UserTrackerFrameListener()
{
}

NiTE2UserTrackerFrameListener::~NiTE2UserTrackerFrameListener()
{
}

void NiTE2UserTrackerFrameListener::onNewFrame(nite::UserTracker& userTracker)
{
  if ( userTracker.isValid() )
  {
    nite::Status niteRc;    
    niteRc = userTracker.readFrame(&userTrackerFrame_);
    if (niteRc != nite::STATUS_OK)
    {
      ROS_ERROR("nite::UserTracker: get next frame failed");
    }
    else
    {
      if ( userTrackerFrame_.isValid() )
      {
        callback_(userTrackerFrame_, userTracker);
      }
      else
        ROS_WARN("Skipping user tracker frame as it is not valid");
    }
  }
}

}
