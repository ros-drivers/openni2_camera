

#include "openni2_camera/openni2_device.h"
#include "NiTE-2/NiTE.h"

namespace nite2_wrapper
{

  class  NiTE2HandTrackerFrameListener: public nite::HandTracker::NewFrameListener {

  public:  
    NiTE2HandTrackerFrameListener();
    virtual ~NiTE2HandTrackerFrameListener();

    void onNewFrame(nite::HandTracker&);

    void setCallback(HandTrackerFrameCallbackFunction& callback)
    {
      callback_ = callback;
    }

  private:

    nite::HandTrackerFrameRef handTrackerFrame_;    
    HandTrackerFrameCallbackFunction callback_;
  };

}
