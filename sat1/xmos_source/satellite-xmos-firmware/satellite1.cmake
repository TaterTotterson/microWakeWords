query_tools_version()

foreach(FFVA_AP ${FFVA_PIPELINES_INT})

    set(FFVA_INT_COMPILE_DEFINITIONS
    ${APP_COMPILE_DEFINITIONS}
        appconfEXTERNAL_MCLK=0
        appconfI2S_ENABLED=1
        appconfUSB_ENABLED=1
        appconfUSB_AUDIO_ENABLED=0
        appconfUSB_AUDIO_MODE=0
        appconfUSB_CDC_ENABLED=0
        appconfAEC_REF_DEFAULT=appconfAEC_REF_I2S
        appconfI2S_MODE=appconfI2S_MODE_MASTER
        appconfI2S_AUDIO_SAMPLE_RATE=48000
        appconfDEVICE_CTRL_SPI=1
    )

    if(${FFVA_AP} STREQUAL bypass )
      set(PL_NAME fixed_delay)
      list(APPEND FFVA_INT_COMPILE_DEFINITIONS appconfPIPELINE_BYPASS=1)
    else()
      set(PL_NAME ${FFVA_AP})
      list(APPEND FFVA_INT_COMPILE_DEFINITIONS appconfPIPELINE_BYPASS=0)
    endif()

    # message(${FFVA_INT_COMPILE_DEFINITIONS})
    
    #**********************
    # Tile Targets
    #**********************
    set(TARGET_NAME tile0_satellite1_firmware_${FFVA_AP})
    add_executable(${TARGET_NAME} EXCLUDE_FROM_ALL)
    target_sources(${TARGET_NAME} PUBLIC ${APP_SOURCES})
    target_include_directories(${TARGET_NAME} PUBLIC ${APP_INCLUDES})
    target_compile_definitions(${TARGET_NAME}
        PUBLIC
            ${FFVA_INT_COMPILE_DEFINITIONS}
            THIS_XCORE_TILE=0
    )
    target_compile_options(${TARGET_NAME} PRIVATE ${APP_COMPILER_FLAGS})
    target_link_libraries(${TARGET_NAME}
        PUBLIC
            ${APP_COMMON_LINK_LIBRARIES}
            fph::ffva::satellite1
            fph::ffva::ap::${PL_NAME}
            sln_voice::app::ffva::sp::passthrough
    )
    target_link_options(${TARGET_NAME} PRIVATE ${APP_LINK_OPTIONS})
    unset(TARGET_NAME)

    set(TARGET_NAME tile1_satellite1_firmware_${FFVA_AP})
    add_executable(${TARGET_NAME} EXCLUDE_FROM_ALL)
    target_sources(${TARGET_NAME} PUBLIC ${APP_SOURCES})
    target_include_directories(${TARGET_NAME} PUBLIC ${APP_INCLUDES})
    target_compile_definitions(${TARGET_NAME}
        PUBLIC
            ${FFVA_INT_COMPILE_DEFINITIONS}
            THIS_XCORE_TILE=1
    )
    target_compile_options(${TARGET_NAME} PRIVATE ${APP_COMPILER_FLAGS})
    target_link_libraries(${TARGET_NAME}
        PUBLIC
            ${APP_COMMON_LINK_LIBRARIES}
            fph::ffva::satellite1
            fph::ffva::ap::${PL_NAME}
            sln_voice::app::ffva::sp::passthrough
    )
    target_link_options(${TARGET_NAME} PRIVATE ${APP_LINK_OPTIONS})
    unset(TARGET_NAME)

    #*********************
    # Create version.h
    #*********************
    SET(VERSIONING_CMD "build")
    if(USE_DEV_TRACKING)
        list(APPEND VERSIONING_CMD "--track")
    endif()

    add_custom_target(satellite1_firmware_${FFVA_AP}_versioning
        COMMAND ${Python3_EXECUTABLE} ${VERSIONING_SCRIPT} ${VERSIONING_CMD} satellite1_firmware_${FFVA_AP}
        COMMENT "Running versioning.py build satellite1_firmware_${FFVA_AP}"
        VERBATIM
    )
    add_dependencies(tile0_satellite1_firmware_${FFVA_AP} satellite1_firmware_${FFVA_AP}_versioning)
    add_dependencies(tile1_satellite1_firmware_${FFVA_AP} satellite1_firmware_${FFVA_AP}_versioning)

    #**********************
    # Merge binaries
    #**********************    
    merge_binaries(satellite1_firmware_${FFVA_AP} tile0_satellite1_firmware_${FFVA_AP} tile1_satellite1_firmware_${FFVA_AP} 1)

    #**********************
    # Create run and debug targets
    #**********************
    create_run_target(satellite1_firmware_${FFVA_AP})
    create_debug_target(satellite1_firmware_${FFVA_AP})
    create_upgrade_img_target(satellite1_firmware_${FFVA_AP} ${XTC_VERSION_MAJOR} ${XTC_VERSION_MINOR})
    
    #**********************
    # Create data partition support targets
    #**********************
    set(TARGET_NAME satellite1_firmware_${FFVA_AP})
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
        
    create_flash_image_target(
        #[[ Target ]]                  ${TARGET_NAME}
        #[[ Boot Partition Size ]]     0x100000
    #   #[[ Data Partition Contents ]] ${DATA_PARTITION_FILE}
    #   #[[ Dependencies ]]            ${DATA_PARTITION_FILE}

    )
    create_flash_app_target(
        #[[ Target ]]                  ${TARGET_NAME}
        #[[ Boot Partition Size ]]     0x100000
        #[[ Data Partition Contents ]] ${DATA_PARTITION_FILE}
        #[[ Dependencies ]]            ${DATA_PARTITION_FILE}
    )

    unset(DATA_PARTITION_FILE_LIST)
endforeach()
