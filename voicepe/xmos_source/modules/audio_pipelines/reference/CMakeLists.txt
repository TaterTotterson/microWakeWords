##******************************************
## Create fixed_delay AEC+IC+NS+AGC 
##   2 mic input channels
#    2 reference input channels
##******************************************

add_library(fixed_delay_aec_ic_ns_agc_2mic_2ref INTERFACE)
target_sources(fixed_delay_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay/audio_pipeline_t0.c
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay/audio_pipeline_t1.c
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay/aec/aec_process_frame_1thread.c
)
target_include_directories(fixed_delay_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay
)
target_link_libraries(fixed_delay_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
        fwk_voice::aec
        fwk_voice::agc
        fwk_voice::ic
        fwk_voice::ns
        fwk_voice::vnr::features
        fwk_voice::vnr::inference
)

##******************************************
## Create ADEC "prevarch" AEC+IC+NS+AGC 
##   2 mic input channels
#    2 reference input channels
##******************************************

add_library(adec_aec_ic_ns_agc_2mic_2ref INTERFACE)
target_sources(adec_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/adec/audio_pipeline_t0.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/audio_pipeline_t1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/stage1/delay_buffer.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/stage1/stage_1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/aec/aec_process_frame_1thread.c
)
target_include_directories(adec_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/adec
        ${CMAKE_CURRENT_LIST_DIR}/adec/aec
        ${CMAKE_CURRENT_LIST_DIR}/adec/stage1
)
target_link_libraries(adec_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
        fwk_voice::adec
        fwk_voice::aec
        fwk_voice::agc
        fwk_voice::ic
        fwk_voice::ns
        fwk_voice::vnr::features
        fwk_voice::vnr::inference
)

##******************************************
## Create ADEC altarch AEC+IC+NS+AGC 
##   2 mic input channels
#    2 reference input channels
##******************************************

add_library(adec_altarch_aec_ic_ns_agc_2mic_2ref INTERFACE)
target_sources(adec_altarch_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/audio_pipeline_t0.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/audio_pipeline_t1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/stage1/delay_buffer.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/stage1/stage_1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/aec/aec_process_frame_1thread.c
)
target_include_directories(adec_altarch_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/aec
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/stage1
)
target_link_libraries(adec_altarch_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
        fwk_voice::adec
        fwk_voice::aec
        fwk_voice::agc
        fwk_voice::ic
        fwk_voice::ns
        fwk_voice::vnr::features
        fwk_voice::vnr::inference
)

##******************************************
## Create empty audio pipeline
##******************************************

add_library(empty_2mic_2ref INTERFACE)
target_sources(empty_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/empty/audio_pipeline.c
)
target_include_directories(empty_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/empty
)
target_link_libraries(empty_2mic_2ref
    INTERFACE
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
)

##*********************************************
## Create aliases for sln_voice example designs
##*********************************************

add_library(sln_voice::app::ffva::ap::adec_altarch ALIAS adec_altarch_aec_ic_ns_agc_2mic_2ref)
add_library(sln_voice::app::ffva::ap::fixed_delay ALIAS fixed_delay_aec_ic_ns_agc_2mic_2ref)
add_library(sln_voice::app::ffva::ap::adec ALIAS adec_aec_ic_ns_agc_2mic_2ref)
add_library(sln_voice::app::ffva::ap::empty ALIAS empty_2mic_2ref)
