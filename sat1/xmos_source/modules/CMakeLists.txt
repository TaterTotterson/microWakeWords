## Add frameworks
add_subdirectory(core)

add_subdirectory(io)
add_subdirectory(lib_qspi_fast_read)
add_subdirectory(rtos)
add_subdirectory(fph)

if(${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS3A)
    ## Need to guard so host targets will not be built
    add_subdirectory(voice)
    add_subdirectory(inferencing)
    add_subdirectory(sw_pll/lib_sw_pll)

    ## The following alias is added to support in intermediate version of fwk_voice
    ##  This can be removed once fwk_voice is updated to use the new, core::lib_tflite_micro alias
    add_library(sdk::inferencing::lib_tflite_micro ALIAS inferencing_tflite_micro)
endif()

## Add additional modules
add_subdirectory(sample_rate_conversion)
add_subdirectory(xscope_fileio)