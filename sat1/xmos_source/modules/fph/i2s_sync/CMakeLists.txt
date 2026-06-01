
if((${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS3A) OR (${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS2A))
    ## Create library target
    add_library(fph_i2s_sync INTERFACE)
    target_sources(fph_i2s_sync
        INTERFACE
            src/rtos_i2s.c
            src/rtos_i2s_rpc.c
    )
    target_include_directories(fph_i2s_sync
        INTERFACE
            api
    )
    target_link_libraries(fph_i2s_sync
        INTERFACE
            lib_i2s
            rtos::osal
    )

    ## Create an alias
    add_library(fph::i2s_sync ALIAS fph_i2s_sync)
endif()
