cmake_minimum_required(VERSION 3.2)
project(toolchain_test)

enable_language(CXX C ASM)

add_executable(c c.c)
add_executable(xc xc.xc)
add_executable(cpp cpp.cpp)
add_executable(s s.S)
add_executable(lowers lowers.S)
add_executable(ass ass.ass)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL XCORE_XS3A)
    set(TARGET XCORE-AI-EXPLORER)
else()
    set(TARGET XCORE-200-EXPLORER)
endif()

include (CTest)

foreach(app c xc cpp s lowers ass)
    target_compile_options(${app} PRIVATE -target=${TARGET})
    target_link_options(${app} PRIVATE -target=${TARGET})

    add_test(NAME ${app} COMMAND xsim $<TARGET_FILE:${app}>)
endforeach()
