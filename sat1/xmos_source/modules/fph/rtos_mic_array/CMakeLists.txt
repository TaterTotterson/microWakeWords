if(DEFINED FRAMEWORK_RTOS_ROOT_PATH)
    set(LIB_RTOS_MIC_ARRAY_PATH ${FRAMEWORK_RTOS_ROOT_PATH}/modules/drivers/mic_array)
else()
    message(FATAL_ERROR "FRAMEWORK_IO_ROOT_PATH was not defined")
endif()

# Note, the inclusion of the vanilla API stuff below is a temporary hack until
# a more RTOS-friendly API is available for the mic array
if((${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS3A) OR (${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS2A))
    ## Create library target
    add_library(fph_rtos_drivers_mic_array INTERFACE)
    target_sources(fph_rtos_drivers_mic_array
        INTERFACE
            ${LIB_RTOS_MIC_ARRAY_PATH}/src/rtos_mic_array.c
            ${LIB_RTOS_MIC_ARRAY_PATH}/src/rtos_mic_array_rpc.c
            vanilla/mic_array_vanilla.cpp
    )
    target_include_directories(fph_rtos_drivers_mic_array
        INTERFACE
            ${LIB_RTOS_MIC_ARRAY_PATH}/api
            vanilla/
    )
    target_link_libraries(fph_rtos_drivers_mic_array
        INTERFACE
            lib_mic_array
            rtos::osal
    )
    target_compile_definitions(fph_rtos_drivers_mic_array
        INTERFACE
            MIC_ARRAY_BASIC_API_ENABLE=1
    )

    ## Create an alias
    add_library(fph::rtos_mic_array ALIAS fph_rtos_drivers_mic_array)
endif()
