.. _sln_voice_system_prerequisites_programming:

#############
Prerequisites
#############

It is recommended that you download and install the latest release of the `XTC Tools <https://www.xmos.com/software/tools/>`__.  XTC Tools 15.2.1 or newer are required for building, running, flashing and debugging the example applications.

`CMake 3.21 <https://cmake.org/download/>`_ or newer and `Git <https://git-scm.com/>`_ are also required for building the example applications.

*******
Windows
*******

A standard C/C++ compiler is required to build applications for the host PC.  Windows users may use `Build Tools for Visual Studio <https://docs.microsoft.com/en-us/cpp/build/building-on-the-command-line?view=msvc-170#download-and-install-the-tools>`__ command-line interface.

It is highly recommended to use *Ninja* as the build system for native Windows firmware builds.
To install *Ninja* follow install instructions at https://ninja-build.org/ or on Windows
install with ``winget`` by running the following commands in *PowerShell*:

.. code-block:: PowerShell

    # Install
    winget install Ninja-build.ninja
    # Reload user Path
    $env:Path=[System.Environment]::GetEnvironmentVariable("Path","User")

XCORE-VOICE host builds should also work using other Windows GNU development environments like GNU Make, MinGW or Cygwin.

libusb
======

The DFU feature of XCORE-VOICE requires `dfu-util <https://dfu-util.sourceforge.net/>`_.

*****
macOS
*****

A standard C/C++ compiler is required to build applications for the host PC.  Mac users may use the Xcode command-line tools.
