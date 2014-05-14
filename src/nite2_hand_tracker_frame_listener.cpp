#include "openni2_camera/nite2_hand_tracker_frame_listener.h"

#include "NiTE-2/NiTE.h"

#include <ros/ros.h>

namespace nite2_wrapper
{

NiTE2HandTrackerFrameListener::NiTE2HandTrackerFrameListener()
{
}

NiTE2HandTrackerFrameListener::~NiTE2HandTrackerFrameListener()
{
}

void NiTE2HandTrackerFrameListener::onNewFrame(nite::HandTracker& handTracker)
{
  if ( handTracker.isValid() )
  {
    nite::Status niteRc;    
    niteRc = handTracker.readFrame(&handTrackerFrame_);
    if (niteRc != nite::STATUS_OK)
    {
      ROS_ERROR("nite::HandTracker: get next frame failed");
    }
    else
    {
      if ( handTrackerFrame_.isValid() )
      {
        callback_(handTrackerFrame_);
      }
      else
        ROS_WARN("Skipping hand tracker frame as it is not valid");
    }
  }
}

}
