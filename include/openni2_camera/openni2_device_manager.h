
/*
 * driver.h
 *
 *  Created on: Apr 8, 2013
 *      Author: jkammerl
 */

#ifndef OPENNI2_DEVICE_MANAGER_H_
#define OPENNI2_DEVICE_MANAGER_H_

#include "openni2_camera/openni2_device_info.h"

#include <boost/thread/mutex.hpp>

#include <vector>
#include <string>
#include <ostream>

namespace openni2_wrapper
{

class OpenNI2DeviceListener;
class OpenNI2Device;

class OpenNI2DeviceManager
{
public:
  OpenNI2DeviceManager();
  virtual ~OpenNI2DeviceManager();

  static boost::shared_ptr<OpenNI2DeviceManager> getSingelton();

  boost::shared_ptr<std::vector<OpenNI2DeviceInfo> > getConnectedDeviceInfos() const;
  boost::shared_ptr<std::vector<std::string> > getConnectedDeviceURIs() const;
  std::size_t getNumOfConnectedDevices() const;

  boost::shared_ptr<OpenNI2Device> getAnyDevice();
  boost::shared_ptr<OpenNI2Device> getDevice(const std::string& device_URI);

protected:
  boost::shared_ptr<OpenNI2DeviceListener> device_listener_;

  static boost::shared_ptr<OpenNI2DeviceManager> singelton_;
};


std::ostream& operator <<(std::ostream& stream, const OpenNI2DeviceManager& device_manager);

}

#endif
