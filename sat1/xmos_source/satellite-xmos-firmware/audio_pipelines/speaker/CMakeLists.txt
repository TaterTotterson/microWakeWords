##******************************************
## Create empty audio pipeline
##******************************************

add_library(empty_2ch_pt INTERFACE)
target_sources(empty_2ch_pt
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/passthrough/speaker_pipeline.c
)
target_include_directories(empty_2ch_pt
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/passthrough
)
target_link_libraries(empty_2ch_pt
    INTERFACE
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
)


##*********************************************
## Create aliases for sln_voice example designs
##*********************************************

add_library(sln_voice::app::ffva::sp::passthrough ALIAS empty_2ch_pt)
