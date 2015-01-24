^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openni2_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
