openni2_camera
==============

ROS wrapper for openni 2.2 and NiTE 2.2

Note: openni2_camera supports xtion devices, but not kinects. For using a kinect with ROS, try the freenect stack: http://www.ros.org/wiki/freenect_stack

This package derives from https://github.com/OpenNI/OpenNI2 and includes NiTE 2.2 to perform:

* hand tracking and gesture recognition (waving and clicking)
* person detection and tracking
* skeleton tracking


Requirements
============

Linux Ubuntu 12.04 32-bit with ROS hydro
----------------------------------------

* OpenNI-Linux-x86-2.2 (version 2.2.0.33)
* NiTE-Linux-x86-2.2   (version 2.2.0.10)
* pal_msgs
* openni2_launch

### Installation

#### OpenNI-2.2

Right now the following files are alread included so you may skip the following steps:

From OpenNI-Linux-x86-2.2.0.33.tar.zip: 

Copy Redist/libOpenNI2.so to openni2_camera/lib  
Copy Redist/OpenNI2 to openni2_camera/lib  
Copy Include/Linux-x86 to openni2_camera/include/OpenNI-2  
Copy Include/Driver to openni2_camera/include/OpenNI-2  

#### NiTE-2.2

Right now the following files are alread included so you may skip the following steps:

From NiTE-Linux-x86-2.2.tar.zip:

Copy libNiTE2.so to openni2_camera/lib  
Copy Include/* to openni2_camera/include/NiTE-2  

#### pal_msgs


Clone the following repository in your catkin workspace:

```
git clone git@github.com:pal-robotics/pal_msgs.git
cd pal_msgs
git checkout hydro-devel
```

#### openni2_launch

Clone the following repository in your catkin workspace:

```
git clone https://github.com/ros-drivers/openni2_launch.git
cd openni2_launch
git checkout hydro-devel
```

How to
======

How to launch the camera node
-----------------------------

```
roslaunch openni2_launch openni2.launch
```

How to visualize the rgb image
------------------------------

```
rosrun image_view image_view image:=/camera/rgb/image_raw
```

How to visualize the depth image
--------------------------------

```
rosrun image_view image_view image:=/camera/depth/image
```

How to test gesture recognition
-------------------------------

```
rostopic echo /camera/gestures
```

Wave in front of the camera or do the 'click' movement, i.e. point forward and bring back the arm


How to visualize the user map
-----------------------------

The detected and tracked persons can be visualized in a segmented depth image as follows

```
rosrun image_view image_view image:=/camera/user_map
```

<img align="middle" src="https://raw.github.com/pal-robotics/openni2_camera/add_user_tracker/etc/user_map.jpg"/>





