
option(DEBUG_FFVA_USB_MIC_INPUT        "Enable ffva usb mic input"  OFF)
option(DEBUG_FFVA_USB_MIC_INPUT_PIPELINE_BYPASS  "Enable ffva usb mic input and audio pipeline bypass"  OFF)

set(FFVA_UA_COMPILE_DEFINITIONS
    ${APP_COMPILE_DEFINITIONS}
    appconfI2S_ENABLED=1
    appconfUSB_ENABLED=1
    appconfAEC_REF_DEFAULT=appconfAEC_REF_USB
    appconfI2S_MODE=appconfI2S_MODE_MASTER

    MIC_ARRAY_CONFIG_MCLK_FREQ=24576000
)

if(DEBUG_FFVA_USB_MIC_INPUT)
    list(APPEND FFVA_UA_COMPILE_DEFINITIONS appconfMIC_SRC_DEFAULT=appconfMIC_SRC_USB)
    list(APPEND FFVA_UA_COMPILE_DEFINITIONS appconfUSB_AUDIO_MODE=appconfUSB_AUDIO_TESTING)
endif()

# a usb mic enabled build without the pipeline for sample rate conversion testing
if(DEBUG_FFVA_USB_MIC_INPUT_PIPELINE_BYPASS)
    list(APPEND FFVA_UA_COMPILE_DEFINITIONS appconfMIC_SRC_DEFAULT=appconfMIC_SRC_USB)
    list(APPEND FFVA_UA_COMPILE_DEFINITIONS appconfUSB_AUDIO_MODE=appconfUSB_AUDIO_TESTING)
    list(APPEND FFVA_UA_COMPILE_DEFINITIONS appconfPIPELINE_BYPASS=1)
    list(APPEND FFVA_UA_COMPILE_DEFINITIONS appconfUSB_AUDIO_SAMPLE_RATE=48000)
endif()

query_tools_version()
foreach(FFVA_AP ${FFVA_PIPELINES_UA})
    #**********************
    # Tile Targets
    #**********************
    set(TARGET_NAME tile0_example_ffva_ua_dev_${FFVA_AP})
    add_executable(${TARGET_NAME} EXCLUDE_FROM_ALL)
    target_sources(${TARGET_NAME} PUBLIC ${APP_SOURCES})
    target_include_directories(${TARGET_NAME} PUBLIC ${APP_INCLUDES})
    target_compile_definitions(${TARGET_NAME}
        PUBLIC
            ${FFVA_UA_COMPILE_DEFINITIONS}
            THIS_XCORE_TILE=0
    )
    target_compile_options(${TARGET_NAME} PRIVATE ${APP_COMPILER_FLAGS})
    target_link_libraries(${TARGET_NAME}
        PUBLIC
            ${APP_COMMON_LINK_LIBRARIES}
            sln_voice::app::ffva::nc_voice_kit
            sln_voice::app::ffva::ap::${FFVA_AP}
    )
    target_link_options(${TARGET_NAME} PRIVATE ${APP_LINK_OPTIONS})
    unset(TARGET_NAME)

    set(TARGET_NAME tile1_example_ffva_ua_dev_${FFVA_AP})
    add_executable(${TARGET_NAME} EXCLUDE_FROM_ALL)
    target_sources(${TARGET_NAME} PUBLIC ${APP_SOURCES})
    target_include_directories(${TARGET_NAME} PUBLIC ${APP_INCLUDES})
    target_compile_definitions(${TARGET_NAME}
        PUBLIC
            ${FFVA_UA_COMPILE_DEFINITIONS}
            THIS_XCORE_TILE=1
    )
    target_compile_options(${TARGET_NAME} PRIVATE ${APP_COMPILER_FLAGS})
    target_link_libraries(${TARGET_NAME}
        PUBLIC
            ${APP_COMMON_LINK_LIBRARIES}
            sln_voice::app::ffva::nc_voice_kit
            sln_voice::app::ffva::ap::${FFVA_AP}
    )
    target_link_options(${TARGET_NAME} PRIVATE ${APP_LINK_OPTIONS})
    unset(TARGET_NAME)

    #**********************
    # Merge binaries
    #**********************
    merge_binaries(example_ffva_ua_dev_${FFVA_AP} tile0_example_ffva_ua_dev_${FFVA_AP} tile1_example_ffva_ua_dev_${FFVA_AP} 1)

    #**********************
    # Create run and debug targets
    #**********************
    create_run_target(example_ffva_ua_dev_${FFVA_AP})
    create_debug_target(example_ffva_ua_dev_${FFVA_AP})
    create_upgrade_img_target(example_ffva_ua_dev_${FFVA_AP} ${XTC_VERSION_MAJOR} ${XTC_VERSION_MINOR})

    #**********************
    # Create data partition support targets
    #**********************
    set(TARGET_NAME example_ffva_ua_dev_${FFVA_AP})
    set(DATA_PARTITION_FILE ${TARGET_NAME}_data_partition.bin)
    set(FATFS_FILE ${TARGET_NAME}_fat.fs)
    set(FATFS_CONTENTS_DIR ${TARGET_NAME}_fatmktmp)

    add_custom_target(
        ${FATFS_FILE} ALL
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${FATFS_CONTENTS_DIR}/fs/
        COMMAND ${CMAKE_COMMAND} -E make_directory ${FATFS_CONTENTS_DIR}/fs/
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/filesystem_support/demo.txt ${FATFS_CONTENTS_DIR}/fs/
        COMMAND fatfs_mkimage --input=${FATFS_CONTENTS_DIR} --output=${FATFS_FILE}
        COMMENT
            "Create filesystem"
        VERBATIM
    )

    set_target_properties(${FATFS_FILE} PROPERTIES
        ADDITIONAL_CLEAN_FILES ${FATFS_CONTENTS_DIR}
    )

    # The filesystem is the only component in the data partition, copy it to
    # the assocated data partition file which is required for CI.
    add_custom_command(
        OUTPUT ${DATA_PARTITION_FILE}
        COMMAND ${CMAKE_COMMAND} -E copy ${FATFS_FILE} ${DATA_PARTITION_FILE}
        DEPENDS
            ${FATFS_FILE}
        COMMENT
            "Create data partition"
        VERBATIM
    )

    list(APPEND DATA_PARTITION_FILE_LIST
        ${FATFS_FILE}
        ${DATA_PARTITION_FILE}
    )

    create_data_partition_directory(
        #[[ Target ]]                   ${TARGET_NAME}
        #[[ Copy Files ]]               "${DATA_PARTITION_FILE_LIST}"
        #[[ Dependencies ]]             "${DATA_PARTITION_FILE_LIST}"
    )

    create_flash_app_target(
        #[[ Target ]]                  ${TARGET_NAME}
        #[[ Boot Partition Size ]]     0x100000
        #[[ Data Partition Contents ]] ${DATA_PARTITION_FILE}
        #[[ Dependencies ]]            ${DATA_PARTITION_FILE}
    )

    unset(DATA_PARTITION_FILE_LIST)
endforeach()
