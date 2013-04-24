/*
 * driver.h
 *
 *  Created on: Apr 8, 2013
 *      Author: jkammerl
 */

#ifndef OPENNI2_DEVICE_INFO_H_
#define OPENNI2_DEVICE_INFO_H_

#include <ostream>

#include <boost/cstdint.hpp>

namespace openni2_wrapper
{

struct OpenNI2DeviceInfo
{
  std::string uri_;
  std::string vendor_;
  std::string name_;
  uint16_t vendor_id_;
  uint16_t product_id_;
};

std::ostream& operator << (std::ostream& stream, const OpenNI2DeviceInfo& device_info);

}

#endif /* DRIVER_H_ */
