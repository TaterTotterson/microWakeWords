
if((${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS3A) OR (${CMAKE_SYSTEM_NAME} STREQUAL XCORE_XS2A))
    ## Source files
    file(GLOB_RECURSE LIB_C_SOURCES   lib_src/lib_src/src/*.c )
    file(GLOB_RECURSE LIB_CXX_SOURCES lib_src/lib_src/src/*.cc)
    file(GLOB_RECURSE LIB_XC_SOURCES  lib_src/lib_src/src/*.xc)
    file(GLOB_RECURSE LIB_ASM_SOURCES lib_src/lib_src/src/*.S )

    ## cmake doesn't recognize .S files as assembly by default
    set_source_files_properties(LIB_ASM_SOURCES PROPERTIES LANGUAGE ASM)

    ## Assume all asm is XS3A for now
    set(XCORE_XS3A_SOURCES ${LIB_ASM_SOURCES})

    ## Gather library sources
    set(LIB_PUBLIC_SOURCES   "")
    set(LIB_PRIVATE_SOURCES  ${LIB_C_SOURCES} ${LIB_CXX_SOURCES} ${LIB_XC_SOURCES})

    ## Append platform specific sources
    list(APPEND LIB_PRIVATE_SOURCES ${${CMAKE_SYSTEM_NAME}_SOURCES})

    ## Create library target
    add_library(lib_src STATIC)
    target_sources(lib_src
        PRIVATE
            ${LIB_PRIVATE_SOURCES}
    )
    target_include_directories(lib_src
        PUBLIC
            lib_src/lib_src/api
            lib_src/lib_src/src/fixed_factor_of_3
            lib_src/lib_src/src/fixed_factor_vpu_voice
            lib_src/lib_src/src/fixed_factor_of_3/ds3
            lib_src/lib_src/src/fixed_factor_of_3/os3
            lib_src/lib_src/src/fixed_factor_of_3_voice
            lib_src/lib_src/src/fixed_factor_of_3_voice/ds3_voice
            lib_src/lib_src/src/fixed_factor_of_3_voice/us3_voice
            lib_src/lib_src/src/multirate_hifi
            lib_src/lib_src/src/multirate_hifi/asrc
            lib_src/lib_src/src/multirate_hifi/ssrc
    )
    target_link_libraries(lib_src
        PUBLIC
            framework_core_legacy_compat
    )

    ## Set any local library compile options
    set(LIB_COMPILE_FLAGS "-O3")

    target_compile_options(lib_src
    PRIVATE
        ${LIB_COMPILE_FLAGS}
    )
endif()
