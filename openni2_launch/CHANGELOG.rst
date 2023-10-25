^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openni2_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2023-10-25)
------------------
* Fix: make subprocess output a string instead of bytes, needed in python 3.7 `#113 <https://github.com/ros-drivers/openni2_camera/issues/113>`_

1.6.0 (2022-04-12)
------------------
* [capability] Add depth_registered and depth_registered_filtered arguments `#111 <https://github.com/ros-drivers/openni2_camera/issues/111>`_ 

1.5.1 (2021-02-01)
------------------
* Default branch update for ROS1 development.
* [CI] Add GitHub Actions configs

1.5.0 (2021-01-12)
------------------
* [capability] Use serialnumber in camera names `#101 <https://github.com/ros-drivers/openni2_camera/issues/101>`_
* Release for Noetic `#98 <https://github.com/ros-drivers/openni2_camera/issues/98>`_

1.4.2 (2020-06-03)
------------------
* update package.xml for noetic support
* Contributors: Michael Ferguson

0.4.2 (2018-12-01)
------------------
* Fix roswtf plugin dependency `#92 <https://github.com/ros-drivers/openni2_camera/issues/92>`_
* Contributors: Isaac I.Y. Saito

0.4.1 (2018-10-20)
------------------
* [fix] [roswtf plugin] Accept vendor and product IDs (fix `#88 <https://github.com/ros-drivers/openni2_camera/issues/88>`_).
* [improve] Remove unused ROS Parameters. `#76 <https://github.com/ros-drivers/openni2_camera/issues/76>`_
* [improve] Add roslaunch check.
* Contributors: Isaac I.Y. Saito

0.4.0 (2018-07-17)
------------------
* Add a simple roswtf plugin. `#80 <https://github.com/ros-drivers/openni2_camera/issues/80>`_
* Contributors: Isaac I.Y. Saito, PlusOne Robotics Inc.

0.3.0 (2017-11-03)
------------------
* Move openni2_launch package into openni2_camera repository `#55 <https://github.com/ros-drivers/openni2_camera/issues/55>`_
* Add rosdoc-based document.
* Contributors: Isaac I.Y. Saito

0.2.2 (2015-01-23)
------------------
* add tf_prefix version, as per `#16 <https://github.com/ros-drivers/openni2_launch/issues/16>`_
* fix `#19 <https://github.com/ros-drivers/openni2_launch/issues/19>`_, reverts `#16 <https://github.com/ros-drivers/openni2_launch/issues/16>`_
* Contributors: Michael Ferguson

0.2.1 (2014-05-22)
------------------
* Force device_id to string type
* Contributors: Dariush Forouher, Michael Ferguson

0.2.0 (2014-05-20)
------------------
* Remove machine arg, as it is not necessary.
* Add tf_prefix same as openni
* Defaults for depth processing are set apropriately for both hardware and software registration.
* Contributors: Libor Wagner, Mark Pitchless, Michael Ferguson, Piyush Khandelwal

0.1.2 (2013-09-30)
------------------
* Expose driver parameters. Note: depth_registration is now off by default.

0.1.1 (2013-09-25)
------------------
* Properly namespace the nodelet manager

0.1.0 (2013-09-10)
------------------
* First release
* This package is a thin wrapper around rgbd_launch
