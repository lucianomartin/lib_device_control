.. include:: ../../README.rst

|newpage|

Overview
--------

The application uses a total of four logical cores which can be seen in the diagram below. Two logical cores take care of implementing USB stack
(one for low level transactions and one for handling Endpoint 0 control requests). One further logical core is used to run the button and LED server task.
The fourth logical core runs the application and communicates with both the button/LED server task and EP0 from where
it receives and sends and receives commands over the Device Control API from the host.

Block diagram
.............

.. figure:: images/block_diagram.pdf
   :width: 100%

   Application core diagram

When implementing a USB device, a device class must be specified. The Device Control protocol is device class agnostic
and instead of using class request type a vendor request type (bmRequestType) is used. For the purpose of
this example a vendor specific device class/subclass/protocol of value 0xff/0xff/0xff is used. This allows the host to enumerate the device
which is required if vendor requests are to be made.

How to use the Device Control library
-------------------------------------

The Makefile
............

To start using the device control library add ``lib_device_control`` to your Makefile::

  USED_MODULES = .. lib_device_control ...

The application in this application note also uses the USB Core, Device and Shared libraries (``module_xud module_usb_device module_usb_shared``) for access to USB and
the Mic Array Board Support library (``lib_mic_array_board_support``) for access to the buttons
and LEDs on the hardware. So the Makefile also includes::

  USED_MODULES = .. lib_usb lib_mic_array_board_support ..


Includes
........

This application requires the system header that defines XMOS xCORE specific defines for declaring and initializing hardware:

.. literalinclude:: main.xc
   :start-on: include <platform.h>
   :end-before: include "app.h"

The Device Control library library functions and types are defined in ``control.h``. This header must
be included in your code to use the library. The low level USB device functionality is provided by ``usb.h``
and the USB device functionality is provided by the API described in ``hid.h`` and it's associated descriptor
set in ``descriptors.h``. Access to the xCORE Array Microphone board GPIO is provided by ``mic_array_board_support.h``.

Allocating hardware resources
.............................

On an xCORE the pins are controlled by ``ports``. The Device Control library itself requires no ports.
However the application uses USB and GPIO. The USB I/O resources are implicitly defined by including the USB library
whereas the buttons and LEDs ports are explicitly passed to the I/O server task. The specific resource IDs are
defined in ``mic_array_board_support.h`` which themselves are defined in the platform file ``vfspk_base.xn``:

.. literalinclude:: main.xc
   :start-on: on tile[0]: mabs_led_ports_t p_leds
   :end-on: on tile[0]: in port p_buttons

Registering Controllable Resources
..................................

Before any control can take place the controlled entity must register controllable resources. This is done by populating
a table of resource IDs at startup which is done by ``register_resources()``. The ``register_resources()`` method is only called once at initialization time. For this
application, only one controllable resource is registered.

.. literalinclude:: app.xc
   :start-on: case i_control.register_resources
   :end-on: break;

Reading over Device Control
...........................

When the host requests a read from a controlled resource, endpoint 0 receives a device-to-host vendor request.

.. literalinclude:: main.xc
   :start-on: case USB_BMREQ_D2H_VENDOR_DEV
   :end-on: res = XUD_DoGetRequest(ep0_out, ep0_in, request_data, len, len);

The Device Control ``control_process_usb_get_request()`` function takes the USB parameters
``wIndex``, ``wValue`` and ``wLength`` along with a reference to the data buffer and converts it into a
method call across the interface ``i_control``. A call to the ``read_command()`` method is subsequently made on the server side control interface which is then handled by the application. Once the application has filled the buffer with data and completed it's ``select`` case, the data buffer reference is returned to xud by EP0 and the USB control transaction is completed. If the application does not indicate a successful filling of the buffer then the vendor request transaction is stalled to indicate failure to the host.

.. literalinclude:: app.xc
   :start-on: case i_control.read_command
   :end-before: printf(

Writing over Device Control
...........................

When the host requests a write to a controlled resource, a similar packet to read is sent to endpoint 0,
which is a host-to-device vendor request.

.. literalinclude:: main.xc
   :start-on: case USB_BMREQ_H2D_VENDOR_DEV:
   :end-on: res = XUD_DoSetRequestStatus(ep0_in);

The Device Control ``control_process_usb_get_request()`` function takes the USB parameters
``wIndex``, ``wValue`` and ``wLength`` along with a reference to the data buffer and converts it into an
method call across the interface ``i_control``. The call to the ``write_command()`` method is subsequently made on the control interface which is then handled by server side in the application. Once the application has handled the passed data and has completed it's ``select`` case a return code is returned. The USB transaction is then either be acknowledged or stalled by the EP0 handler to indicate success or failure to the host.

.. literalinclude:: app.xc
   :start-on: case i_control.write_command
   :end-before: printf(


The application main() function
...............................

The ``main()`` function sets up the tasks within the application.

Firstly, the ``interfaces`` and ``channels`` are declared. xC channels provide a simple way of
passing data tokens between concurrent tasks, without the need to worry about switch route setup
or the low level control token protocol. ``XUD`` is written using a channel interface and so uses
this method of communicating.

.. literalinclude:: main.xc
   :start-on: chan c_ep_out[NUM_EP_OUT], c_ep_in[NUM_EP_IN];
   :end-before: interface control i_control[1];

xC Interfaces also provide a means of concurrent tasks communicating with each other.
Interfaces add high level language features on top of the channels and allow remote
calling of methods while passing parameters and returning values, all with the benefit of type checking.

Communication between the Endpoint 0 task and the LED and Button server is performed using
interfaces.

.. literalinclude:: main.xc
   :start-on: interface control i_control[1];
   :end-before: par

The rest of the ``main()`` function starts all the tasks in parallel using the xC ``par`` construct:

.. literalinclude:: main.xc
   :start-on: par
   :end-before: return 0

Note that ``xud()`` and ``endpoint0()`` are placed on ``USB_TILE`` which is ``tile[1]`` (as defined in the Makefile) leaving
``tile[0]`` completely free for the application. The ``mabs_button_and_led_server()`` task needs
to be placed on ``tile[0]`` because the GPIO connected to the LEDs and buttons reside on that
tile. The Application task app() may be place on either tile.

This code starts all of the tasks concurrently and they then communicate over the channels and interfaces.

The application app() task
..........................

The application here is simple. It consists of a while(1) select{} block containing three cases.
These cases are outlined above and handle registration, reading and writing events triggered by the control library.
It waits for calls to these cases from the client side (Endpoint 0) and handles them accordingly by registering the controlled
resource, setting LEDs or reading the buttons.

Please see the source code listing app_listing_ for more details.

.. TODO: Check how the substitution below worked and update it.
.. |appendix|

Demo Hardware Setup
-------------------

To run the demo, connect a USB cable to power the Vocal Fusion board
and plug the xTAG to the board and connect the xTAG USB cable to your development machine.

.. figure:: images/hw_setup.pdf
   :width: 80%

   Hardware setup

|newpage|

Launching the demo application
------------------------------

Once the demo example has been built either from the command line using xmake from the project directory
where the Makefile can be found or via the build mechanism of xTIMEcomposer studio it can be executed on the Vocal Fusion board.

Once built there will be a ``bin/`` directory within the project which contains
the binary for the xCORE device. The xCORE binary has a XMOS standard .xe extension.

Launching from the command line
...............................

From the command line use the ``xrun`` tool to download and run the code
on the xCORE device::

  xrun --xscope bin/AN01034_usb.xe

Once this command has executed the application will be running on the
Vocal Fusion board.

Launching from xTIMEcomposer Studio
...................................

From xTIMEcomposer Studio use the run mechanism to download code to xCORE device.
Select the xCORE binary from the ``bin/`` directory, right click and go to **Run
Configurations**. Double click on xCORE application to create a new run configuration,
enable the xSCOPE I/O mode in the dialog box and then select **Run**.

Once this command has executed the application will be running on the Vocal Fusion board.

Running the application
.......................

Once the application is started using either of the above methods there should
be the following output printed to the console::

  started

The device is now ready to receive and handle control requests from the host.

Building the host Application
.............................

The host application is supported on Windows and OSX platforms. To build the application on a Windows platform, it
is expected that the Visual Studio compiler is installed (cl.exe). On the OSX platform, the Xcode command line
tools must be installed.

To build the application, from a suitable command line shell with the compiler environment set, type either for Windows::

  nmake -f Makefile.Win32

or for OSX::

  make -f Makefile.OSX

The build will result in a binary being compiled into bin/a.out.

When running under a Windows platform, additionally a driver will be required to fully enumerate the USB device. See
the ``Intsalling the Driver on Windows`` section driver_ for details.

Running the host application
............................

To run under windows::

  \bin\a.exe

To run under OSX::

  ./bin/a.out

The application will attempt to enumerate the device and, on success, will request the user to input a number. This
number will be sent across the device control API as a write command and set an appropriate number of LEDs on the device hardware.
A number between 0 and 13 is valid. For example::

  device found
  started
  Enter number of LEDs to be lit: 11

After receiving the command and the appropriate number of LEDs are lit, the last button event will be reported. The buttons are active low so the following report
indicates that the last button event was button C being released::

  Last button event: C value: 1

|newpage|

.. _driver:

Installing the Driver on Windows
................................

The test vendor specific device used in this application which requires a driver on Windows. The driver is installed
by finding the device when the application is running and connected to the host PC. Select :menuitem:`Control Panel,Device Manager`

.. figure:: images/driver_1.png
   :width: 40%

   Device before driver install

Right click on the device and select **Update Driver Software....**

.. figure:: images/driver_2.png
   :width: 40%

   Update the driver

|newpage|

Select **Browse for driver software on your computer** and choose the ``libusb\Win32\driver`` directory.

.. figure:: images/driver_3.png
   :width: 40%

   Locate the driver

The driver should be installed and create a device called **XMOS Custom Device**.

.. figure:: images/driver_4.png
   :width: 40%

   Once the driver has been installed

The device is now ready to be controlled on Windows.


|newpage|

References
----------

.. nopoints::

  * XMOS Tools User Guide

    http://www.xmos.com/published/xtimecomposer-user-guide

  * XMOS xCORE Programming Guide

    http://www.xmos.com/published/xmos-programming-guide

  * XMOS Device Control Library

    http://www.xmos.com/support/libraries/lib_device_control

  * XMOS USB Library (and it's associated application notes)

    http://www.xmos.com/support/libraries/lib_usb

|newpage|

Full source code listing
------------------------

Source code for main.xc
.......................

.. TODO: Check if the language below is correct.
.. literalinclude:: ../src/main.xc
  :language: c

.. _app_listing:

Source code for app.xc
.......................

.. TODO: Check if the language below is correct.
.. literalinclude:: ../src/app.xc
  :language: c

|newpage|
