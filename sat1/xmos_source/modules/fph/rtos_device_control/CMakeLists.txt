
if((${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS3A) OR (${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS2A))
    ## Create library target
    add_library(fph_rtos_sw_services_device_control INTERFACE)
    target_sources(fph_rtos_sw_services_device_control
        INTERFACE
            src/device_control.c
            src/resource_table.c
            transport/i2c/device_control_i2c.c
            transport/usb/device_control_usb.c
            transport/spi/device_control_spi.c
    )
    target_include_directories(fph_rtos_sw_services_device_control
        INTERFACE
            api
            transport/i2c
            transport/usb
            transport/spi
    )
    target_link_libraries(fph_rtos_sw_services_device_control
        INTERFACE
            rtos::osal
            rtos::drivers::i2c
            rtos::drivers::spi
            rtos::sw_services::usb
    )

    ## Create an alias
    add_library(fph::device_control ALIAS fph_rtos_sw_services_device_control)
else()
    ## Host app
    add_library(fph_rtos_sw_services_device_control_host_usb INTERFACE)

    # Discern OS for libusb library location
    if ((${CMAKE_SYSTEM_NAME} MATCHES "Darwin") AND (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64"))
        target_link_directories(fph_rtos_sw_services_device_control_host_usb INTERFACE "host/libusb/OSX64")
        set(libusb-1.0_INCLUDE_DIRS "host/libusb/OSX64")
        set(LINK_LIBS usb-1.0.0)
    elseif ((${CMAKE_SYSTEM_NAME} MATCHES "Darwin") AND (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64"))
        target_link_directories(fph_rtos_sw_services_device_control_host_usb INTERFACE "host/libusb/OSXARM")
        set(libusb-1.0_INCLUDE_DIRS "host/libusb/OSXARM")
        set(LINK_LIBS usb-1.0.0)
    elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        find_package(PkgConfig)
        pkg_check_modules(libusb-1.0 REQUIRED libusb-1.0)
        set(LINK_LIBS usb-1.0)
    elseif (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        target_link_directories(fph_rtos_sw_services_device_control_host_usb INTERFACE "host/libusb/Win32")
        set(libusb-1.0_INCLUDE_DIRS "host/libusb/Win32")
        set(LINK_LIBS libusb)
    endif()

    target_sources(fph_rtos_sw_services_device_control_host_usb
        INTERFACE
            host/util.c
            host/device_access_usb.c
    )
    target_include_directories(fph_rtos_sw_services_device_control_host_usb
        INTERFACE
            api
            host
            ${libusb-1.0_INCLUDE_DIRS}
    )
    target_compile_definitions(fph_rtos_sw_services_device_control_host_usb INTERFACE USE_USB=1)
    target_link_libraries(fph_rtos_sw_services_device_control_host_usb
        INTERFACE
            ${LINK_LIBS}
    )

    ## Create an alias
    add_library(fph::host::device_control_host_usb ALIAS fph_rtos_sw_services_device_control_host_usb)
endif()
