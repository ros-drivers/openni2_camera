#!/usr/bin/env python3

# Copyright (c) 2020-2023, Michael Ferguson
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of the copyright holder nor the names of its
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

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    namespace_param_name = "namespace"
    namespace = LaunchConfiguration(namespace_param_name)
    namespace_launch_arg = DeclareLaunchArgument(namespace_param_name, default_value='camera')

    tf_prefix_param_name = "tf_prefix"
    tf_prefix = LaunchConfiguration(tf_prefix_param_name)
    tf_prefix_launch_arg = DeclareLaunchArgument(tf_prefix_param_name, default_value='')

    container = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver
                launch_ros.descriptions.ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace=namespace,
                    parameters=[{'depth_registration': False},
                                {'use_device_time': True},
                                {'rgb_frame_id': [namespace,"_rgb_optical_frame"]},
                                {'depth_frame_id': [namespace,"_depth_optical_frame"]},
                                {'ir_frame_id': [namespace,"_ir_optical_frame"]},],
                    remappings=[('depth/image', 'depth_registered/image_raw')],
                ),
                # Create XYZ point cloud
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='points_xyz',
                    namespace=namespace,
                    parameters=[{'queue_size': 10}],
                    remappings=[('image_rect', 'depth/image_raw'),
                                ('camera_info', 'depth/camera_info'),
                                ('points', 'depth/points')],
                ),
            ],
            output='screen',
    )

    tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ThisLaunchFileDir(), "tfs.launch.py"])),
        launch_arguments={namespace_param_name: namespace, tf_prefix_param_name: tf_prefix}.items(),
    )

    return launch.LaunchDescription([
        namespace_launch_arg,
        tf_prefix_launch_arg,
        container,
        tfs,
    ])
