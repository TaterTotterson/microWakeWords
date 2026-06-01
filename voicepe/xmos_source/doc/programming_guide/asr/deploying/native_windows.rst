
******************************************
Deploying the Firmware with Native Windows
******************************************

This document explains how to deploy the software using *CMake* and *Ninja*. If you are not using native Windows MSVC build tools and instead using a Linux emulation tool, refer to :ref:`sln_voice_asr_deploying_linux_macos_programming_guide`.

To install *Ninja* follow install instructions at https://ninja-build.org/ or on Windows
install with ``winget`` by running the following commands in *PowerShell*:

.. code-block:: PowerShell

    # Install
    winget install Ninja-build.ninja
    # Reload user Path
    $env:Path=[System.Environment]::GetEnvironmentVariable("Path","User")

Building the Host Server
========================

This application requires a host application to serve files to the device. The served file must be named ``test.wav``.  This filename is defined in ``src/app_conf.h``.

Run the following commands in the root folder to build the host application using your native Toolchain:

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
  ninja xscope_host_endpoint
  ninja install

The host application, ``xscope_host_endpoint.exe``, will install at ``<USERPROFILE>\.xmos\bin``, and may be moved if desired.  You may wish to add this directory to your ``PATH`` variable.

Before running the host application, you may need to add the location of ``xscope_endpoint.dll`` to your ``PATH``. This environment variable will be set if you run the host application in the XTC Tools command-line environment.  For more information see `Configuring the command-line environment <https://www.xmos.ai/documentation/XM-014363-PC-LATEST/html/tools-guide/install-configure/getting-started.html>`__.

Building the Firmware
=====================

Run the following commands in the root folder to build the firmware:

.. code-block:: console

    cmake -G Ninja -B build --toolchain=xmos_cmake_toolchain/xs3a.cmake
    cd build
    ninja example_asr

.. _sln_voice_asr_programming_guide_flash_model:

Flashing the Model
==================

The model file is part of the data partition file.  The data partition file includes a file used to calibrate the flash followed by the model.

Run the following commands in the build folder to create the data partition:

.. code-block:: console

    ninja make_data_partition_example_asr

Then run the following commands in the build folder to flash the data partition:

.. code-block:: console

    xflash --force --quad-spi-clock 50MHz --target-file ../examples/speech_recognition/XK_VOICE_L71.xn --write-all example_asr_data_partition.bin

Running the Firmware
====================

From the build folder run:

.. code-block:: console

    xrun --xscope-realtime --xscope-port localhost:12345 example_asr.xe

In a second console, run the following command in the ``examples/speech_recognition`` folder to run the host server:

.. code-block:: console

    xscope_host_endpoint.exe 12345
