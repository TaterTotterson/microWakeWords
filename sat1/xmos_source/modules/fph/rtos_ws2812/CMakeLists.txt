
if((${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS3A) OR (${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS2A))
    ## Create library target
    add_library(rtos_ws2812 INTERFACE)
    target_sources(rtos_ws2812
        INTERFACE
            src/rtos_ws2812.c
    )
    target_include_directories(rtos_ws2812
        INTERFACE
            api
    )
    target_link_libraries(rtos_ws2812
        INTERFACE
            rtos::osal
    )

    ## Create an alias
    add_library(fph::rtos_ws2812 ALIAS rtos_ws2812)
endif()
