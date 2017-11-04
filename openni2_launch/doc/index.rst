.. openni2_launch documentation master file, created by
   sphinx-quickstart on Fri Nov  3 15:35:05 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. toctree::
   :maxdepth: 2

==========
Usage
==========

Simplest use
------------

This package contains several `launch` files, but `openni.launch` serves as a single point of entry. It can be started by::

  roslaunch openni2_launch openni.launch

Note that with this command the camera driver only accesses a single device. So run as many launch files as needed with correct arguments if you're running multiple devices.

Options/arguments
------------------

`openni.launch` takes the following options.

* `device_id`: can have the following formats:

  * `#1`  : the first device found

  * `2@X` : the Xth device on USB bus 2.

E.g. When 2 openni2-based cameras are connected and `lsusb -t` returns the following, where Dev 18 and 19 are the cameras, `device_id` can be `3@18` and `3@19`::

  $ lsusb -t
  /:  Bus 03.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/14p, 480M
      |__ Port 2: Dev 18, If 0, Class=Vendor Specific Class, Driver=usbfs, 480M
      |__ Port 2: Dev 18, If 1, Class=Audio, Driver=, 480M
      |__ Port 2: Dev 18, If 2, Class=Audio, Driver=, 480M
      |__ Port 3: Dev 15, If 0, Class=Human Interface Device, Driver=usbhid, 1.5M
      |__ Port 3: Dev 15, If 1, Class=Human Interface Device, Driver=usbhid, 1.5M
      |__ Port 6: Dev 19, If 0, Class=Vendor Specific Class, Driver=usbfs, 480M
      |__ Port 6: Dev 19, If 1, Class=Audio, Driver=, 480M
      |__ Port 6: Dev 19, If 2, Class=Audio, Driver=, 480M

TBD

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
