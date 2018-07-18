# Software License Agreement (BSD License)
#
# Copyright (c) 2018, PlusOne Robotics, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Plus One Robotics, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import logging
import re
import subprocess

import rospy
from roswtf.rules import warning_rule, error_rule


def _device_notfound_subproc(id_manufacturer, id_product):
    """
    @rtype: [dict]
    @return: Example:

                        [{'device': '/dev/bus/usb/002/004', 'tag': 'Lenovo ', 'id': '17ef:305a'},
                         {'device': '/dev/bus/usb/002/001', 'tag': 'Linux Foundation 3.0 root hub', 'id': '1d6b:0003'},
                         {'device': '/dev/bus/usb/001/006', 'tag': 'Validity Sensors, Inc. ', 'id': '138a:0090'},,,]

    @note: This method depends on Linux command (via subprocess), which makes
                 this command platform-dependent. Ubuntu Xenial onward, a Python module
                 that encapsulate platform operation becomes available so this method
                 can be wiped out. See https://github.com/ros-drivers/openni2_camera/pull/80#discussion_r193295442
    """
    device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
    df = subprocess.check_output("lsusb")
    devices = []
    for i in df.split('\n'):
        if i:
            info = device_re.match(i)
            if info:
                dinfo = info.groupdict()
                logging.debug("dinfo: {}, dinfo.id: {}".format(dinfo, dinfo["id"]))
                if dinfo["id"] == "{}:{}".format(id_manufacturer, id_product):
                    dinfo['device'] = "/dev/bus/usb/{}/{}".format(dinfo.pop('bus'), dinfo.pop('device'))
                    devices.append(dinfo)
    logging.info("#devices: {}\ndevices: {}".format(len(devices), devices))
    return devices


def sensor_notfound(ctx):
    """
    @summary: Check if expected number of sensors are found.
              Expected number of sensors can be set by
              ROS Parameter 'openni2_num_sensors_expected'.
    @note: Technically this can be static check, but because of the
           need for connecting to ROS Param server, this needs
           to be online check.
    """
    errors = []
    num_sensors_expected = rospy.get_param("openni2_num_sensors_expected", 1)
    # The set of manufacture id and prod id. Default: Asus Xtion.
    id_manufacturer = rospy.get_param("id_manufacturer", "1d27")
    id_product = rospy.get_param("id_product", "0601")
    devices = _device_notfound_subproc(
        id_manufacturer=id_manufacturer, id_product=id_product)
    num_sensors = len(devices)
    if num_sensors != num_sensors_expected:
        errors.append("{} openni2 sensors found (expected: {}).".format(
            num_sensors, num_sensors_expected))
    return errors


# app_warnings and app_errors declare the rules that we actually check
app_warnings_online = [
]

app_warnings_static = [
]

app_errors_online = [
  (sensor_notfound, "Different number of openni2 sensors found."),
]

app_errors_static = [
]


# roswtf entry point for online checks
def roswtf_plugin_online(ctx):
    for r in app_warnings_online:
        warning_rule(r, r[0](ctx), ctx)
    for r in app_errors_online:
        error_rule(r, r[0](ctx), ctx)


def roswtf_plugin_static(ctx):
    for r in app_warnings_static:
        warning_rule(r, r[0](ctx), ctx)
    for r in app_errors_static:
        error_rule(r, r[0](ctx), ctx)
