# Cmake Toolchain for XMOS xcore processor tools.

## Overview

This repo contains toolchain files that allow building of xcore-200 (xs2a architecture) and xcore.ai (xs3a architecture) targets under `cmake`.
This repo is usually included as a submodule within a project and the ``xs2a.cmake`` or ``xs3a.cmake`` file is passed to the `cmake` build command using the 
```-DCMAKE_TOOLCHAIN_FILE``` argument. The following is an example of this which creates the make files in the ```/build``` sub-directory:

```
cmake -B build -DCMAKE_TOOLCHAIN_FILE=modules/xmos_cmake_toolchain/xs3a.cmake
```

If using Windows and `NMake` pass the ```-G "NMake Makefiles"``` argument or if using Windows and `Ninja` pass the ```-G "Ninja"``` argument additionally to the above `cmake` command. Typically `Ninja` has been found to build much faster and is the recommended build system for Windows hosts.

Once the `cmake` stage has completed the actual compilation build can commence. This is typically done using `make` on Linux or Mac hosts and with `NMake` or `Ninja` on Windows hosts. Ensure you are running this command from the ```/build``` directory specified in the `cmake` stage.

## Tests

A test suite checks the defined toolchains against a few cmake versions to ensure they build and run as expected. To run the tests locally do:

```
pip install -Ur requirements.txt
cd test
tox run
```
