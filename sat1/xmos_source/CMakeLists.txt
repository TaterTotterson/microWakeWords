cmake_minimum_required(VERSION 3.21)

## Disable in-source build.
if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "In-source build is not allowed! Please specify a build folder.\n\tex:cmake -B build")
endif()

## Project declaration
project(satellite-xmos-firmware)

## Enable languages for project
enable_language(CXX C ASM)

## Setup a root path
set(XMOS_FIRMWARE_ROOT_PATH ${PROJECT_SOURCE_DIR} CACHE STRING "Root folder of xmos_firmware in this cmake project tree")

## Import xmos macros
include(xmos_macros.cmake)

## Add frameworks
add_subdirectory(modules)

## Add top level project targets
if(PROJECT_IS_TOP_LEVEL AND ${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS3A)
    include(satellite-xmos-firmware/firmware.cmake)
endif()

