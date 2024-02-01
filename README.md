# openni2_camera

## Introduction
ROS wrapper for openni 2.0

Note: openni2_camera supports xtion devices, but not kinects. For using a kinect with ROS, try the freenect stack: http://www.ros.org/wiki/freenect_stack

## Contribution

Branching:
- ROS1:
   - Latest: [ros1](https://github.com/ros-drivers/openni2_camera/tree/ros1)
   - For ROS [Jade](http://wiki.ros.org/jade), [Indigo](http://wiki.ros.org/indigo) or earlier: [indigo-devel](https://github.com/ros-drivers/openni2_camera/tree/indigo-devel)
- ROS2:
   - The [ros2](https://github.com/ros-drivers/openni2_camera/tree/ros2) branch supports Jazzy and later
   - The [iron](https://github.com/ros-drivers/openni2_camera/tree/iron) branch supports Humble to Iron
   - openni2_launch has NOT been ported since proper lazy subscribers are still not possible in ROS2

## Developer document
   - [docs.ros.org/openni2_launch](http://docs.ros.org/en/melodic/api/openni2_launch/html/)
   - Source of the doc: [openni2_launch/doc](./openni2_launch/doc/)

## Running ROS2 Driver

An example launch exists that loads just the camera component:

```
ros2 launch openni2_camera camera_only.launch.py
```

If you want to get a PointCloud2, use:

```
ros2 launch openni2_camera camera_with_cloud.launch.py
```

## Migration from ROS1

 * The rgb/image topic has been renamed to rgb/image_raw for consistency.
 * The nodelet has been refactored into an rclcpp component called
   "openni2_wrapper::OpenNI2Driver". See the launch folder for an example
   of how to start this.
 * Since most components in image_proc/depth_image_proc lack lazy pub/sub,
   the advanced processing graphs in rgbd_launch and openni2_launch are not
   currently feasible. It is recommended to create a launch file with the
   specific pipeline you want. See the launch folder for an example.

## Known Issues

 * There are currently no subscriber connect/disconnect callbacks in ROS2.
   This package implements a lazy publisher by running a 1Hz update loop
   and seeing if there are new subscribers.
 * Using "use_device_time" is currently broken.
