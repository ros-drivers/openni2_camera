^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openni2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (2025-05-21)
------------------
* Replace ament_target_dependencies with target_link_libraries (`#145 <https://github.com/ros-drivers/openni2_camera/issues/145>`_)
  `ament_target_dependencies` is deprecated, it will require a release on
  `rolling`
* Contributors: Alejandro Hernández Cordero

2.2.2 (2024-11-03)
------------------
* add dynamic parameters (`#141 <https://github.com/ros-drivers/openni2_camera/issues/141>`_)
  In ROS 1, a number of things were dynamic - this PR re-adds them using
  the new ROS 2 paradigm for parameter updating. I've tested this with a
  Primesense device and am able to disable auto_exposure and
  auto_white_balance.
* Contributors: Michael Ferguson

2.2.1 (2024-08-11)
------------------
* rename depth/image_raw to depth_raw/image (`#140 <https://github.com/ros-drivers/openni2_camera/issues/140>`_)
  Without this change, both depth/image and depth/image_raw publish the
  same camera_info topic (depth/camera_info) - this has several issues:
  * subscribing to either image causes both to be published along with
  their respective camera_info
  * remapping either camera_info, remaps both
  * In Jazzy, it appears that the double camera_info also causes minor
  issues with message_filters in downstream packages
* Contributors: Michael Ferguson

2.2.0 (2024-03-14)
------------------
* fix build on Ubuntu Noble (24.04) (`#138 <https://github.com/ros-drivers/openni2_camera/issues/138>`_)
  add missing header
* Contributors: Michael Ferguson

2.1.0 (2024-02-14)
------------------
* implement proper lazy subscribers (`#136 <https://github.com/ros-drivers/openni2_camera/issues/136>`_)
  Fixes `#119 <https://github.com/ros-drivers/openni2_camera/issues/119>`_, works in Jazzy and later
* publish tfs and set matching frame names (`#135 <https://github.com/ros-drivers/openni2_camera/issues/135>`_)
  This PR ports the TFs from original
  [`kinect_frames.launch`](https://github.com/ros-drivers/rgbd_launch/blob/noetic-devel/launch/kinect_frames.launch)
  to the ROS 2 launch files and sets the image frame names appropriately.
* Depth-only point cloud launch file for Non-RGB PrimeSense device. (`#132 <https://github.com/ros-drivers/openni2_camera/issues/132>`_)
* Contributors: Christian Rauch, Michael Ferguson, TinLethax

2.0.2 (2023-04-26)
------------------
* add depth_image_proc exec dep (`#131 <https://github.com/ros-drivers/openni2_camera/issues/131>`_)
  Needed when starting the camera with point cloud processing.
* Contributors: Mario Prats

2.0.1 (2023-03-02)
------------------
* fix device time (`#129 <https://github.com/ros-drivers/openni2_camera/issues/129>`_)
* add build depend on pkg-config (`#128 <https://github.com/ros-drivers/openni2_camera/issues/128>`_)
* Contributors: Michael Ferguson

2.0.0 (2023-03-01)
------------------
* revive the test wrapper (`#127 <https://github.com/ros-drivers/openni2_camera/issues/127>`_)
  resolves `#123 <https://github.com/ros-drivers/openni2_camera/issues/123>`_
* remove boost and unused files (`#122 <https://github.com/ros-drivers/openni2_camera/issues/122>`_)
  * replace boost functionality by standard C++
  * remove ROS nodelet metadata
  * store parameter callback
  * remove ROS1 node
  * remove unused os import
  Co-authored-by: Christian Rauch <Rauch.Christian@gmx.de>
* fix build, target only humble and later (`#118 <https://github.com/ros-drivers/openni2_camera/issues/118>`_)
* Capability: [CI] Add GitHub Action (`#116 <https://github.com/ros-drivers/openni2_camera/issues/116>`_)
  * Capability: [CI] Add GitHub Action
  * Maintenance: Update maintaner contact
* initial port to ROS2
* Contributors: Isaac I.Y. Saito, Michael Ferguson

1.6.0 (2022-04-12)
------------------

1.5.1 (2021-02-01)
------------------

1.5.0 (2021-01-12)
------------------
* [capability] Use serialnumber in camera names `#101 <https://github.com/ros-drivers/openni2_camera/issues/101>`_
* Release for Noetic `#98 <https://github.com/ros-drivers/openni2_camera/issues/98>`_

1.4.2 (2020-06-03)
------------------

0.4.2 (2018-12-01)
------------------

0.4.1 (2018-10-20)
------------------

0.4.0 (2018-07-17)
------------------

0.3.0 (2017-11-03)
------------------


0.2.9 (2017-10-25)
------------------
* [fix] Device re-connection `#53 <https://github.com/ros-drivers/openni2_camera/issues/53>`_
* [fix] Publish projector/camera_info (fixes disparity img) `#48 <https://github.com/ros-drivers/openni2_camera/issues/48>`_
* Contributors: Isaac I.Y. Saito, Martin Guenther, Shaun Edwards

0.2.8 (2017-08-17)
------------------
* [capability] Add exposure time control `#46 <https://github.com/ros-drivers/openni2_camera/issues/46>`_
* [fix] device URI formatting `#47 <https://github.com/ros-drivers/openni2_camera/issues/47>`_
* Contributors: Martin Guenther, Michael Ferguson, Sergey Alexandrov

0.2.7 (2016-06-07)
------------------
* Merge pull request `#44 <https://github.com/ros-drivers/openni2_camera/issues/44>`_ from jacquelinekay/fix_kinetic_build
  fix compile on kinetic
* Contributors: Jackie Kay, Michael Ferguson

0.2.6 (2016-05-05)
------------------
* [fix] Compile for OSX `#30 <https://github.com/ros-drivers/openni2_camera/issues/30>`_
* [fix] Crash on OSX and warning fixes.
* Contributors: Hans Gaiser, Isaac I.Y. Saito, Michael Ferguson

0.2.5 (2015-10-15)
------------------
* Merge pull request `#34 <https://github.com/ros-drivers/openni2_camera/issues/34>`_ from Intermodalics/feature/get_serial_service
  added get_serial service
* Contributors: Michael Ferguson, Ruben Smits

0.2.4 (2015-04-06)
------------------
* proper usage of device_id parameter in resolveDeviceURI, resolve unique parts of device URIs, too
* print vendor id and product id as hex value (like in lsusb)
* Contributors: Michael Ferguson, Stephan Wirth

0.2.3 (2015-01-16)
------------------
* Explicitly ask for serial number when trying to resolve device URI instead of doing it once on device connected, fixes `#24 <https://github.com/ros-drivers/openni2_camera/issues/24>`_
* Contributors: Michael Ferguson, Stephan Wirth

0.2.2 (2014-10-06)
------------------
* Add usb_reset
* Contributors: Kei Okada, Michael Ferguson

0.2.1 (2014-08-22)
------------------
* Fixed a bug that prevents depth only sensors from properly calculating the point cloud due to incorrect focal length
* Updated cmakelists for OSX
* Contributors: Colin Lea, Michael Ferguson, Tarek Taha

0.2.0 (2014-05-22)
------------------
* device_id: find camera by serial number
* Make freenect_stack link a real link for wiki.
* Contributors: Dariush Forouher, Michael Ferguson

0.1.2 (2014-02-03)
------------------
* Fix CMake error.
* Contributors: Benjamin Chretien, Michael Ferguson

0.1.1 (2013-11-13)
------------------
* Fixed default value of ir_mode. Thanks @nxdefiant
  https://github.com/ros-drivers/openni2_camera/issues/16

0.1.0 (2013-08-28)
------------------
* initial release
