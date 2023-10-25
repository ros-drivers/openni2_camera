^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openni2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2023-10-25)
------------------
* Fix: make subprocess output a string instead of bytes, needed in python 3.7 `#113 <https://github.com/ros-drivers/openni2_camera/issues/113>`_

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
