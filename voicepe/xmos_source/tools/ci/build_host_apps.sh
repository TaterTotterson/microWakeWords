#!/bin/bash
set -e

XCORE_VOICE_ROOT=`git rev-parse --show-toplevel`

source ${XCORE_VOICE_ROOT}/tools/ci/helper_functions.sh
export_ci_build_vars

# setup distribution folder
DIST_DIR=${XCORE_VOICE_ROOT}/dist_host
mkdir -p ${DIST_DIR}

# perform builds
path="${XCORE_VOICE_ROOT}"
echo '******************************************************'
echo '* Building host applications'
echo '******************************************************'

(cd ${path}; rm -rf build_host)
(cd ${path}; mkdir -p build_host)
(cd ${path}/build_host; log_errors cmake ../ -G "$CI_CMAKE_GENERATOR"; log_errors $CI_BUILD_TOOL $CI_BUILD_TOOL_ARGS)

# setup configurations
# row format is: "app_name app_path"
host_apps=(
    "fatfs_mkimage              modules/rtos/tools/fatfs_mkimage"
    "datapartition_mkimage      modules/rtos/tools/datapartition_mkimage"
    "xscope_host_endpoint       modules/xscope_fileio/xscope_fileio/host"
    "nibble_swap                modules/lib_qspi_fast_read/tools/nibble_swap"
)

# copy applications to dist
for ((i = 0; i < ${#host_apps[@]}; i += 1)); do
    read -ra FIELDS <<< ${host_apps[i]}
    app_name="${FIELDS[0]}"
    app_path="${FIELDS[1]}"
    (cd ${path}/build_host; cp ${app_path}/${app_name} ${DIST_DIR})
done
