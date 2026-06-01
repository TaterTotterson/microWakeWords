add_library(xscope_fileio INTERFACE)

target_sources(xscope_fileio
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/xscope_fileio/xscope_fileio/src/xscope_io_device.c)

target_include_directories(xscope_fileio
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/xscope_fileio/xscope_fileio
        ${CMAKE_CURRENT_LIST_DIR}/xscope_fileio/xscope_fileio/api)

target_compile_definitions(xscope_fileio
    INTERFACE
        TEST_WAV_XSCOPE=1)
        