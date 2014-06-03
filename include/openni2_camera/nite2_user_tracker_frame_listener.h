

#include "openni2_camera/openni2_device.h"
#include "NiTE-2/NiTE.h"

namespace nite2_wrapper
{

  class  NiTE2UserTrackerFrameListener: public nite::UserTracker::NewFrameListener {

  public:  
    NiTE2UserTrackerFrameListener();
    virtual ~NiTE2UserTrackerFrameListener();

    void onNewFrame(nite::UserTracker&);

    void setCallback(UserTrackerFrameCallbackFunction& callback)
    {
      callback_ = callback;
    }

  private:

    nite::UserTrackerFrameRef userTrackerFrame_;
    UserTrackerFrameCallbackFunction callback_;
  };

}
