#**********************
# Gather Sources
#**********************
file(GLOB_RECURSE APP_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.c )
set(APP_INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/usb
    # ${CMAKE_CURRENT_LIST_DIR}/src/ww_model_runner
    ${CMAKE_CURRENT_LIST_DIR}/src/control
    ${CMAKE_CURRENT_LIST_DIR}/src/dfu_int
    ${CMAKE_CURRENT_LIST_DIR}/src/configuration
)

include(${CMAKE_CURRENT_LIST_DIR}/bsp_config/bsp_config.cmake)

#**********************
# Flags
#**********************
set(APP_COMPILER_FLAGS
    -Os
    -g
    -report
    -fxscope
    -mcmodel=large
    -Wno-xcore-fptrgroup
    ${CMAKE_CURRENT_LIST_DIR}/src/config.xscope
)

set(APP_COMPILE_DEFINITIONS
    DEBUG_PRINT_ENABLE=1
    PLATFORM_USES_TILE_0=1
    PLATFORM_USES_TILE_1=1
    XUD_CORE_CLOCK=600

    CFG_TUSB_DEBUG_PRINTF=rtos_printf
    CFG_TUSB_DEBUG=0
)

set(APP_LINK_OPTIONS
    -lquadspi
    -report
    ${CMAKE_CURRENT_LIST_DIR}/src/config.xscope
)

set(APP_COMMON_LINK_LIBRARIES
    inferencing_tflite_micro
    rtos::freertos_usb
    rtos::sw_services::device_control
    lib_src
)

#**********************
# Pipeline Options
# By default only these targets are created:
#  example_ffva_int_fixed_delay
#  example_ffva_ua_adec_altarch
#**********************
option(ENABLE_ALL_FFVA_PIPELINES  "Create all FFVA pipeline configurations"  OFF)

if(ENABLE_ALL_FFVA_PIPELINES)
    set(FFVA_PIPELINES_INT
        fixed_delay
        adec
        adec_altarch
        empty
    )

    set(FFVA_PIPELINES_UA
        fixed_delay
        adec
        adec_altarch
        empty
    )
else()
    set(FFVA_PIPELINES_INT
        fixed_delay
    )

    set(FFVA_PIPELINES_UA
        adec_altarch
        fixed_delay
    )
endif()

#**********************
# XMOS Example Design Targets
#**********************
include(${CMAKE_CURRENT_LIST_DIR}/ffva_int.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ffva_ua.cmake)

#**********************
# Include FFVA Debug and Extension targets
#**********************
include(${CMAKE_CURRENT_LIST_DIR}/ffva_int_dev.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/ffva_ua_dev.cmake)
