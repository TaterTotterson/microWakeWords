include_guard(DIRECTORY)

find_package(Python3 REQUIRED Interpreter)
## merge_binaries combines multiple xcore applications into one by extracting
## a tile elf and recombining it into another binary.
##
## This macro can be called in two ways. The 4 argument version is for when the
## application has only 1 node and therefore only the core needs to be specified.
##
##    # create target OUT by replacing tile number 0 in BASE with tile 0 in OTHER
##    merge_binaries(${OUT} ${BASE} ${OTHER} 0)
##
## The 5 argument version is for multi-node applications. IMPORTANT: node number 
## is not the "Node Id" from the xn file, rather the index of the node in the 
## JTAGChain which is defined in the xn file.
##
##    # create target OUT by replacing tile 1 on node 0 in BASE with tile 1 on 
##    # node 0 in OTHER
##    merge_binaries(${OUT} ${BASE} ${OTHER} 0 1)
##
function(merge_binaries _OUTPUT_TARGET_NAME _BASE_TARGET _OTHER_TARGET _NODE_OR_TILE_TO_MERGE)
    if(ARGC GREATER 4)
      set(_NODE_NUM_TO_MERGE ${_NODE_OR_TILE_TO_MERGE})
      set(_TILE_NUM_TO_MERGE ${ARGV4})
    else()
      set(_NODE_NUM_TO_MERGE 0)
      set(_TILE_NUM_TO_MERGE ${_NODE_OR_TILE_TO_MERGE})
    endif()

    get_target_property(BASE_TILE_DIR     ${_BASE_TARGET}  BINARY_DIR)
    get_target_property(BASE_TILE_NAME    ${_BASE_TARGET}  NAME)
    get_target_property(OTHER_TILE_NAME   ${_OTHER_TARGET} NAME)

    set(INPUT_ELF ${OTHER_TILE_NAME}_split/image_n${_NODE_NUM_TO_MERGE}c${_TILE_NUM_TO_MERGE}_2.elf)
    add_custom_target(${_OUTPUT_TARGET_NAME} ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory ${OTHER_TILE_NAME}_split
        COMMAND xobjdump --split --split-dir ${OTHER_TILE_NAME}_split ${OTHER_TILE_NAME}.xe > ${OTHER_TILE_NAME}_split/output.log
        # next line fails if file does not exist, there is probably a cheaper way to do this. xobjdump -r silently continues if 
        # the file does not exist
        COMMAND ${CMAKE_COMMAND} -E compare_files ${INPUT_ELF} ${INPUT_ELF}
        COMMAND ${CMAKE_COMMAND} -E copy ${BASE_TILE_NAME}.xe ${_OUTPUT_TARGET_NAME}.xe
        COMMAND xobjdump ${_OUTPUT_TARGET_NAME}.xe -r ${_NODE_NUM_TO_MERGE},${_TILE_NUM_TO_MERGE},${INPUT_ELF} >> ${OTHER_TILE_NAME}_split/output.log
        DEPENDS
            ${_BASE_TARGET}
            ${_OTHER_TARGET}
        BYPRODUCTS
            ${_OUTPUT_TARGET_NAME}.xe
        WORKING_DIRECTORY
            ${BASE_TILE_DIR}
        COMMENT
            "Merge tile ${_NODE_NUM_TO_MERGE},${_TILE_NUM_TO_MERGE} of ${_OTHER_TARGET}.xe into ${_BASE_TARGET}.xe to create ${_OUTPUT_TARGET_NAME}.xe"
        VERBATIM
    )
    set_target_properties(${_OUTPUT_TARGET_NAME} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${BASE_TILE_DIR}
      ADDITIONAL_CLEAN_FILES "${OTHER_TILE_NAME}_split"
    )
endfunction()

## Creates a run target for a provided binary
macro(create_run_target _EXECUTABLE_TARGET_NAME)
    add_custom_target(run_${_EXECUTABLE_TARGET_NAME}
      COMMAND xrun --xscope ${_EXECUTABLE_TARGET_NAME}.xe
      DEPENDS ${_EXECUTABLE_TARGET_NAME}
      COMMENT
        "Run application"
      VERBATIM
    )
endmacro()

## Creates a run target for a provided binary. The first argument specifies the file to save to (no extension).
macro(create_run_xscope_to_file_target _EXECUTABLE_TARGET_NAME _XSCOPE_FILE)
    add_custom_target(run_xscope_to_file_${_EXECUTABLE_TARGET_NAME}
      COMMAND xrun --xscope-file ${_XSCOPE_FILE} ${_EXECUTABLE_TARGET_NAME}.xe
      DEPENDS ${_EXECUTABLE_TARGET_NAME}
      COMMENT
        "Run application"
      VERBATIM
    )
endmacro()

## Creates a debug target for a provided binary
macro(create_debug_target _EXECUTABLE_TARGET_NAME)
    add_custom_target(debug_${_EXECUTABLE_TARGET_NAME}
      COMMAND xgdb ${_EXECUTABLE_TARGET_NAME}.xe -ex "connect" -ex "connect --xscope" -ex "run"
      DEPENDS ${_EXECUTABLE_TARGET_NAME}
      COMMENT
        "Debug application"
    )
endmacro()

## Creates a filesystem file for a provided binary
##   filename must end in "_fat.fs"
## Optional arguments can be used to specify other dependency targets, such as filesystem generators
## create_filesystem_target(_EXECUTABLE_TARGET_NAME _FILESYSTEM_INPUT_DIR _IMAGE_SIZE _OPTIONAL_DEPENDS_TARGETS)
macro(create_filesystem_target)
  if(${ARGC} EQUAL 1)
    add_custom_target(make_fs_${ARGV0} ALL
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/filesystem_support/${ARGV0}_fat.fs ${ARGV0}_fat.fs
      DEPENDS ${ARGV0}_fat.fs
      COMMENT
        "Move filesystem"
      VERBATIM
    )
  elseif(${ARGC} EQUAL 3)
    add_custom_target(make_fs_${ARGV0} ALL
      COMMAND fatfs_mkimage --input=${ARGV1} --image_size=${ARGV2} --output=${ARGV0}_fat.fs
      BYPRODUCTS
        ${ARGV0}_fat.fs
      COMMENT
        "Create filesystem"
      VERBATIM
    )
  elseif(${ARGC} EQUAL 4)
    add_custom_target(make_fs_${ARGV0} ALL
      COMMAND fatfs_mkimage --input=${ARGV1} --image_size=${ARGV2} --output=${ARGV0}_fat.fs
      DEPENDS ${ARGV3}
      BYPRODUCTS
        ${ARGV0}_fat.fs
      COMMENT
        "Create filesystem"
      VERBATIM
    )
  else()
    message(FATAL_ERROR "Invalid number of arguments passed to create_filesystem_target")
  endif()
endmacro()

## Creates a directory populated with all components related to the data partition
##   folder must end in "_data_partition"
## Optional argument can be used to specify dependency targets
## create_data_partition_directory(_EXECUTABLE_TARGET_NAME _FILES_TO_COPY _OPTIONAL_DEPENDS_TARGETS)
macro(create_data_partition_directory)
  if(${ARGC} EQUAL 2)
    add_custom_target(make_data_partition_${ARGV0} ALL
      COMMAND ${CMAKE_COMMAND} -E rm -rf ${ARGV0}_data_partition/
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ARGV0}_data_partition/
        COMMAND ${CMAKE_COMMAND} -E copy ${ARGV1} ${ARGV0}_data_partition/
        COMMENT
            "Collect data partition components"
        VERBATIM
    )
  elseif(${ARGC} EQUAL 3)
    add_custom_target(make_data_partition_${ARGV0} ALL
      COMMAND ${CMAKE_COMMAND} -E rm -rf ${ARGV0}_data_partition/
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ARGV0}_data_partition/
      COMMAND ${CMAKE_COMMAND} -E copy ${ARGV1} ${ARGV0}_data_partition/
      DEPENDS
          ${ARGV2}
      COMMENT
          "Collect data partition components"
      VERBATIM
    )
  else()
    message(FATAL_ERROR "Invalid number of arguments passed to create_data_partition_directory")
  endif()

  set_target_properties(make_data_partition_${ARGV0} PROPERTIES
      ADDITIONAL_CLEAN_FILES ${ARGV0}_data_partition
  )
endmacro()

## Creates a flash app target for a provided binary
## Optional arguments can be used to specify boot partition size, data partition contents, and other dependency targets, such as filesystem generators
## create_flash_app_target(_EXECUTABLE_TARGET_NAME _BOOT_PARTITION_SIZE _DATA_PARTITION_CONTENTS _OPTIONAL_DEPENDS_TARGETS)
function(create_flash_app_target)
  if(${ARGC} EQUAL 1)
    add_custom_target(flash_app_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe
        DEPENDS ${ARGV0}
        COMMENT
          "Flash application"
        USES_TERMINAL
        VERBATIM
      )
  elseif(${ARGC} EQUAL 2)
    add_custom_target(flash_app_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe --boot-partition-size ${ARGV1}
        DEPENDS ${ARGV0}
        COMMENT
          "Flash application with empty data partition"
        USES_TERMINAL
        VERBATIM
      )
  elseif(${ARGC} EQUAL 3)
    add_custom_target(flash_app_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe --boot-partition-size ${ARGV1} --data ${ARGV2}
        DEPENDS ${ARGV0}
        COMMENT
          "Flash application and data partition"
        USES_TERMINAL
        VERBATIM
      )
  elseif(${ARGC} EQUAL 4)
  add_custom_target(flash_app_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe --boot-partition-size ${ARGV1} --data ${ARGV2}
        DEPENDS ${ARGV0} ${ARGV3}
        COMMENT
          "Flash application and data partition"
        USES_TERMINAL
        VERBATIM
    )
  else()
    message(FATAL_ERROR "Invalid number of arguments passed to create_flash_app_target")
  endif()
endfunction()


## Creates a flash image target for a provided binary
## Optional arguments can be used to specify boot partition size, data partition contents, and other dependency targets, such as filesystem generators
## create_flash_image_target(_EXECUTABLE_TARGET_NAME _BOOT_PARTITION_SIZE _DATA_PARTITION_CONTENTS _OPTIONAL_DEPENDS_TARGETS)
function(create_flash_image_target)
  if(USE_DEV_TRACKING)
    set(TRACK_POSTBUILD_COMMAND COMMAND ${Python3_EXECUTABLE} ${VERSIONING_SCRIPT} track ${ARGV0})
  else()
    set(TRACK_POSTBUILD_COMMAND "")
  endif()
  if(${ARGC} EQUAL 1)
    add_custom_target(create_flash_img_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe -o ${ARGV0}.factory.bin
        COMMAND ${Python3_EXECUTABLE} -c "import hashlib; print(hashlib.md5(open('${ARGV0}.factory.bin', 'rb').read()).hexdigest())" > ${ARGV0}.factory.md5
        ${TRACK_POSTBUILD_COMMAND}
        DEPENDS ${ARGV0}
        COMMENT
          "Create factory flash image."
        USES_TERMINAL
        VERBATIM
      )
  elseif(${ARGC} EQUAL 2)
    add_custom_target(create_flash_img_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe --boot-partition-size ${ARGV1} -o ${ARGV0}.factory.bin
        COMMAND ${Python3_EXECUTABLE} -c "import hashlib; print(hashlib.md5(open('${ARGV0}.factory.bin', 'rb').read()).hexdigest())" > ${ARGV0}.factory.md5
        ${TRACK_POSTBUILD_COMMAND}
        DEPENDS ${ARGV0}
        COMMENT
          "Create factory flash image."
        USES_TERMINAL
        VERBATIM
      )
  elseif(${ARGC} EQUAL 3)
    add_custom_target(create_flash_img_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe --boot-partition-size ${ARGV1} --data ${ARGV2} -o ${ARGV0}.factory.bin
        COMMAND ${Python3_EXECUTABLE} -c "import hashlib; print(hashlib.md5(open('${ARGV0}.factory.bin', 'rb').read()).hexdigest())" > ${ARGV0}.factory.md5
        ${TRACK_POSTBUILD_COMMAND}
        DEPENDS ${ARGV0}
        COMMENT
          "Create factory flash image."
        USES_TERMINAL
        VERBATIM
      )
  elseif(${ARGC} EQUAL 4)
  add_custom_target(create_flash_img_${ARGV0}
        COMMAND xflash --quad-spi-clock 50MHz --factory ${ARGV0}.xe --boot-partition-size ${ARGV1} --data ${ARGV2} -o ${ARGV0}.factory.bin
        COMMAND ${Python3_EXECUTABLE} -c "import hashlib; print(hashlib.md5(open('${ARGV0}.factory.bin', 'rb').read()).hexdigest())" > ${ARGV0}.factory.md5
        ${TRACK_POSTBUILD_COMMAND}
        DEPENDS ${ARGV0} ${ARGV3}
        COMMENT
          "Create factory flash image."
        USES_TERMINAL
        VERBATIM
    )
  else()
    message(FATAL_ERROR "Invalid number of arguments passed to create_flash_image_target")
  endif()
endfunction()



## Creates an install target for a provided binary
macro(create_install_target _EXECUTABLE_TARGET_NAME)
    add_custom_target(install_${_EXECUTABLE_TARGET_NAME}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/dist
      COMMAND ${CMAKE_COMMAND} -E copy ${_EXECUTABLE_TARGET_NAME}.xe ${PROJECT_SOURCE_DIR}/dist
      DEPENDS ${_EXECUTABLE_TARGET_NAME}
      COMMENT
        "Install application"
    )
endmacro()

## Creates an xflash image upgrade target for a provided binary
macro(create_upgrade_img_target _EXECUTABLE_TARGET_NAME _FACTORY_MAJOR_VER _FACTORY_MINOR_VER)
    add_custom_target(create_upgrade_img_${_EXECUTABLE_TARGET_NAME}
      COMMAND xflash --factory-version ${_FACTORY_MAJOR_VER}.${_FACTORY_MINOR_VER} --upgrade 0 ${_EXECUTABLE_TARGET_NAME}.xe  -o ${_EXECUTABLE_TARGET_NAME}.upgrade.bin
      COMMAND ${Python3_EXECUTABLE} -c "import hashlib; print(hashlib.md5(open('${_EXECUTABLE_TARGET_NAME}.upgrade.bin', 'rb').read()).hexdigest())" > ${_EXECUTABLE_TARGET_NAME}.upgrade.md5
      DEPENDS ${_EXECUTABLE_TARGET_NAME}
      COMMENT
        "Create upgrade image for application"
      USES_TERMINAL
      VERBATIM
    )
endmacro()

## Creates a loader obj target for a provided binary
## Full filepath must be specified for loader source file
macro(create_loader_target _EXECUTABLE_TARGET_NAME _LOADER_SOURCE_FILE)
    add_custom_target(create_loader_object_${_EXECUTABLE_TARGET_NAME}
      COMMAND xcc -march=xs3a -c ${_LOADER_SOURCE_FILE} -o ${_EXECUTABLE_TARGET_NAME}_loader.o
      DEPENDS
      COMMENT
        "Create loader object file for application"
      VERBATIM
    )
endmacro()

## Creates an xflash erase all target for a provided target XN file
## Full filepath must be specified for XN file
macro(create_erase_all_target _APP_NAME _TARGET_FILEPATH)
    add_custom_target(erase_all_${_APP_NAME}
      COMMAND xflash --erase-all --target-file=${_TARGET_FILEPATH}
      DEPENDS
      COMMENT
        "Erase target flash"
      USES_TERMINAL
      VERBATIM
    )
endmacro()

## Query the version of the XTC Tools
##
##   Populates the following variables:
##
##     XTC_VERSION_MAJOR
##     XTC_VERSION_MINOR
##     XTC_VERSION_PATCH
function(query_tools_version)
    # Run cat "$XMOS_TOOL_PATH"/doc/version.txt
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E cat $ENV{XMOS_TOOL_PATH}/doc/version.txt
        OUTPUT_VARIABLE XCC_VERSION_OUTPUT_STRING
    )
    # Split output semver
    string(FIND ${XCC_VERSION_OUTPUT_STRING} " " SPACE_POSITION)
    string(SUBSTRING ${XCC_VERSION_OUTPUT_STRING} 0 ${SPACE_POSITION} XCC_SEMVER)
    # Parse version fields
    string(REPLACE "." ";" XCC_SEMVER_FIELDS ${XCC_SEMVER})
    list(LENGTH XCC_SEMVER_FIELDS XCC_SEMVER_FIELDS_LENGTH)
    list(GET XCC_SEMVER_FIELDS 0 XCC_VERSION_MAJOR)
    list(GET XCC_SEMVER_FIELDS 1 XCC_VERSION_MINOR)
    list(GET XCC_SEMVER_FIELDS 2 XCC_VERSION_PATCH)
    # Set XTC version env variables
    set(XTC_VERSION_MAJOR ${XCC_VERSION_MAJOR} PARENT_SCOPE)
    set(XTC_VERSION_MINOR ${XCC_VERSION_MINOR} PARENT_SCOPE)
    set(XTC_VERSION_PATCH ${XCC_VERSION_PATCH} PARENT_SCOPE)
endfunction()

