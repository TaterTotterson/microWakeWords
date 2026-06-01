cmake_minimum_required(VERSION 3.21)

## Disable in-source build.
if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "In-source build is not allowed! Please specify a build folder.\n\tex:cmake -B build")
endif()

## Project declaration
project(solution_voice)

## Enable languages for project
enable_language(CXX C ASM)

## Project options
option(XCORE_VOICE_TESTS     "Enable XCORE-VOICE tests"  OFF)

## Setup a root path
set(SOLUTION_VOICE_ROOT_PATH ${PROJECT_SOURCE_DIR} CACHE STRING "Root folder of sln_voice in this cmake project tree")

## Import some helpful macros
include(modules/rtos/tools/cmake_utils/xmos_macros.cmake)

## Add frameworks
add_subdirectory(modules)

## Add top level project targets
if(PROJECT_IS_TOP_LEVEL)
    include(src/ffva.cmake)
endif()

if(XCORE_VOICE_TESTS)
    include(test/tests.cmake)
endif()
