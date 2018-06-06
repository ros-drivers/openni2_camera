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
import re
import subprocess

import rospy
from roswtf.rules import warning_rule, error_rule


def sensor_notfound(ctx):
    """
    @summary: Check if expected number of sensors are found.
                          Expected number of sensors can be set by
                          ROS Parameter 'openni2_num_sensors_expected'.
    """
    errors = []
    num_sensors_expected = rospy.get_param("openni2_num_sensors_expected", 1)
    # TODO: The set of manufacture id and prod id is specific to Asus Xtion.
    #       There may be other openni2-based devices.
    devices = usb.core.find(idVendor=0x1d27, idProduct=0x0601, find_all=True)
    num_sensors = sum(1 for _ in devices)
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
]

app_errors_static = [
  (sensor_notfound, "Different number of openni2 sensors found."),
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
