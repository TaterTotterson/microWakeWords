#!/bin/bash
set -e

# help text
help()
{
   echo "XCORE-VOICE example builder script"
   echo
   echo "Syntax: build_examples.sh [-h] <example_list.txt>"
   echo
   echo "example_list.txt format:"
   echo "   name app_target run_data_partition_target BOARD toolchain"
   echo
   echo "options:"
   echo "h     Print this Help."
}

# flag arguments
while getopts h option
do
    case "${option}" in
        h) help
           exit;;
    esac
done

# get input list file
INPUT_LIST=${@:$OPTIND:1}
if [ ! -f "$INPUT_LIST" ]; then
    echo "ERROR: $INPUT_LIST does not exist"
    exit
fi

XCORE_VOICE_ROOT=`git rev-parse --show-toplevel`

source ${XCORE_VOICE_ROOT}/tools/ci/helper_functions.sh
export_ci_build_vars

# setup distribution folder
DIST_DIR=${XCORE_VOICE_ROOT}/dist
DIST_HOST_DIR=${XCORE_VOICE_ROOT}/dist_host
mkdir -p ${DIST_DIR}

if [ -d "${DIST_HOST_DIR}" ]; then
    # add DIST_HOST_DIR to path.
    #   This is used in CI for fatfs_mkimage
    PATH="${DIST_HOST_DIR}":$PATH
    find ${DIST_HOST_DIR} -type f -exec chmod a+x {} +
fi

readarray examples < ${INPUT_LIST}

# perform builds
for ((i = 0; i < ${#examples[@]}; i += 1)); do
    read -ra FIELDS <<< ${examples[i]}
    name="${FIELDS[0]}"
    app_target="${FIELDS[1]}"
    run_data_partition_target="${FIELDS[2]}"
    board="${FIELDS[3]}"
    toolchain_file="${XCORE_VOICE_ROOT}/${FIELDS[4]}"
    path="${XCORE_VOICE_ROOT}"
    echo '******************************************************'
    echo '* Building' ${name}, ${app_target} 'for' ${board}
    echo '******************************************************'

    (cd ${path}; rm -rf build_${board})
    (cd ${path}; mkdir -p build_${board})
    (cd ${path}/build_${board}; log_errors cmake ../ -G "$CI_CMAKE_GENERATOR" -DCMAKE_TOOLCHAIN_FILE=${toolchain_file} -DBOARD=${board} -DENABLE_ALL_FFVA_PIPELINES=1; log_errors $CI_BUILD_TOOL ${app_target} $CI_BUILD_TOOL_ARGS)
    (cd ${path}/build_${board}; cp ${app_target}.xe ${DIST_DIR})
    if [ "$run_data_partition_target" = "Yes" ]; then
        echo '======================================================'
        echo '= Making data partition for' ${app_target}
        echo '======================================================'
        (cd ${path}/build_${board}; log_errors $CI_BUILD_TOOL make_data_partition_${app_target} $CI_BUILD_TOOL_ARGS)
        (cd ${path}/build_${board}; cp ${app_target}_data_partition.bin ${DIST_DIR})
    fi
    echo '******************************************************'
    echo '* Add extra files'
    echo '******************************************************'
    cd ${path}; cp LICENSE.rst ${DIST_DIR}
    cd ${path}; cp README.rst ${DIST_DIR}
done
