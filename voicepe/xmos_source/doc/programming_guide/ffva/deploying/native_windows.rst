
******************************************
Deploying the Firmware with Native Windows
******************************************

This document explains how to deploy the software using *CMake* and *Ninja*. If you are not using native Windows MSVC build tools and instead using a Linux emulation tool, refer to :ref:`sln_voice_ffva_deploying_linux_macos_programming_guide`.

To install *Ninja* follow install instructions at https://ninja-build.org/ or on Windows
install with ``winget`` by running the following commands in *PowerShell*:

.. code-block:: PowerShell

    # Install
    winget install Ninja-build.ninja
    # Reload user Path
    $env:Path=[System.Environment]::GetEnvironmentVariable("Path","User")

Building the Host Applications
==============================

This application requires a host application to create the flash data partition. Run the following commands in the root folder to build the host application using your native Toolchain:

.. note::

  Permissions may be required to install the host applications.

.. note::

  A C/C++ compiler, such as Visual Studio or MinGW, must be included in the path.

Before building the host application, you will need to add the path to the XTC Tools to your environment.

.. code-block:: console

  set "XMOS_TOOL_PATH=<path-to-xtc-tools>"

Then build the host application:

.. code-block:: console

  cmake -G Ninja -B build_host
  cd build_host
  ninja install

The host applications will be installed at ``%USERPROFILE%\.xmos\bin``, and may be moved if desired.  You may wish to add this directory to your ``PATH`` variable.

Building the Firmware
=====================

Run the following commands in the root folder to build the |I2S| firmware:

.. code-block:: console

    cmake -G Ninja -B build --toolchain=xmos_cmake_toolchain/xs3a.cmake
    cd build
    ninja example_ffva_int_fixed_delay


Run the following commands in the root folder to build the USB firmware:

.. code-block:: console

    cmake -G Ninja -B build --toolchain=xmos_cmake_toolchain/xs3a.cmake
    cd build
    ninja example_ffva_ua_adec_altarch

Running the Firmware
====================

Before the firmware is run, the filesystem must be loaded.

Inside of the build folder root, after building the firmware, run one of:

.. code-block:: console

    ninja flash_app_example_ffva_int_fixed_delay
    ninja flash_app_example_ffva_ua_adec_altarch

Once flashed, the application will run.

After the filesystem has been flashed once, the application can be run without flashing.  If changes are made to the filesystem image, the application must be reflashed.

From the build folder run:

.. code-block:: console

    xrun --xscope example_ffva_int_fixed_delay.xe
    xrun --xscope example_ffva_ua_adec_altarch.xe

Upgrading the Firmware
======================

The UA variants of this application contain DFU over the USB DFU Class V1.1 transport method.

To create an upgrade image from the build folder run:

.. code-block:: console

    ninja create_upgrade_img_example_ffva_ua_adec_altarch

Once the application is running, a USB DFU v1.1 tool can be used to perform various actions.  This example will demonstrate with dfu-util commands.  Installation instructions for respective operating system can be found `here <https://dfu-util.sourceforge.net/>`__

To verify the device is running run:

.. code-block:: console

    dfu-util -l

This should result in an output containing:

.. code-block:: console

    Found DFU: [20b1:4001] ver=0001, devnum=100, cfg=1, intf=3, path="3-4.3", alt=2, name="DFU DATAPARTITION", serial="123456"
    Found DFU: [20b1:4001] ver=0001, devnum=100, cfg=1, intf=3, path="3-4.3", alt=1, name="DFU UPGRADE", serial="123456"
    Found DFU: [20b1:4001] ver=0001, devnum=100, cfg=1, intf=3, path="3-4.3", alt=0, name="DFU FACTORY", serial="123456"

The DFU interprets the flash as 3 separate partitions, the read only factory image, the read/write upgrade image, and the read/write data partition containing the filesystem.

The factory image can be read back by running:

.. code-block:: console

    dfu-util -e -d ,20b1:4001 -a 0 -U readback_factory_img.bin

The factory image can not be written to.

From the build folder, the upgrade image can be written by running:

.. code-block:: console

    dfu-util -e -d ,20b1:4001 -a 1 -D example_ffva_ua_adec_altarch_upgrade.bin

The upgrade image can be read back by running:

.. code-block:: console

    dfu-util -e -d ,20b1:4001 -a 1 -U readback_upgrade_img.bin

On system reboot, the upgrade image will always be loaded if valid.  If the upgrade image is invalid, the factory image will be loaded.  To revert back to the factory image, you can upload an file containing the word 0xFFFFFFFF.

The data partition image can be read back by running:

.. code-block:: console

    dfu-util -e -d ,20b1:4001 -a 2 -U readback_data_partition_img.bin

The data partition image can be written by running:

.. code-block:: console

    dfu-util -e -d ,20b1:4001 -a 2 -D readback_data_partition_img.bin

Note that the data partition will always be at the address specified in the initial flashing call.


Debugging the Firmware
======================

To debug with xgdb, from the build folder run:

.. code-block:: console

    xgdb -ex "connect --xscope" -ex "run" example_ffva_int_fixed_delay.xe
    xgdb -ex "connect --xscope" -ex "run" example_ffva_ua_adec_altarch.xe
