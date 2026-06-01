#!/bin/bash
set -e

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

# setup configurations
# row format is: "name app_target run_data_partition_target flag BOARD toolchain"
tests=(
    "test_asrc_div   test_asrc_div   NONE   NONE   XCORE_AI_EXPLORER   xmos_cmake_toolchain/xs3a.cmake"
    "test_ffva_dfu   example_ffva_ua_adec_altarch   example_ffva_ua_adec_altarch   NONE   XK_VOICE_L71   xmos_cmake_toolchain/xs3a.cmake"
    "test_pipeline_ffd   test_pipeline_ffd   NONE   TEST_PIPELINE=FFD   XK_VOICE_L71   xmos_cmake_toolchain/xs3a.cmake"
    "test_pipeline_ffva_adec_altarch   test_pipeline_ffva_adec_altarch   NONE   TEST_PIPELINE=FFVA_ALT_ARCH   XK_VOICE_L71   xmos_cmake_toolchain/xs3a.cmake"
    "test_asr_sensory   test_asr_sensory   test_asr_sensory   TEST_ASR=SENSORY   XK_VOICE_L71   xmos_cmake_toolchain/xs3a.cmake"
    "test_ffva_sample_rate_conv   example_ffva_ua_adec_altarch   example_ffva_ua_adec_altarch   DEBUG_FFVA_USB_MIC_INPUT_PIPELINE_BYPASS=1   XK_VOICE_L71   xmos_cmake_toolchain/xs3a.cmake"
    "test_ffva_verbose_output   example_ffva_ua_adec_altarch   example_ffva_ua_adec_altarch   DEBUG_FFVA_USB_VERBOSE_OUTPUT=1   XK_VOICE_L71   xmos_cmake_toolchain/xs3a.cmake"
    "test_ffd_gpio   test_ffd_gpio   NONE   NONE   XCORE_AI_EXPLORER   xmos_cmake_toolchain/xs3a.cmake"
    "test_ffd_low_power_audio_buffer   test_ffd_low_power_audio_buffer   NONE   NONE   XK_VOICE_L71   xmos_cmake_toolchain/xs3a.cmake"
)

# perform builds
for ((i = 0; i < ${#tests[@]}; i += 1)); do
    read -ra FIELDS <<< ${tests[i]}
    name="${FIELDS[0]}"
    app_target="${FIELDS[1]}"
    data_partition_target="${FIELDS[2]}"

    flag="${FIELDS[3]}"
    board="${FIELDS[4]}"
    toolchain_file="${XCORE_VOICE_ROOT}/${FIELDS[5]}"
    path="${XCORE_VOICE_ROOT}"
    echo '******************************************************'
    echo '* Building' ${name}, ${app_target} 'for' ${board}
    echo '******************************************************'

    if [ "${flag}" = "NONE" ]; then
        optional_cache_entry=""
    else
        optional_cache_entry="-D${flag}"
    fi

    (cd ${path}; rm -rf build_${board})
    (cd ${path}; mkdir -p build_${board})
    (cd ${path}/build_${board}; log_errors cmake ../ -G "$CI_CMAKE_GENERATOR" -DCMAKE_TOOLCHAIN_FILE=${toolchain_file} -DBOARD=${board} -DXCORE_VOICE_TESTS=1 ${optional_cache_entry}; log_errors $CI_BUILD_TOOL ${app_target} $CI_BUILD_TOOL_ARGS)
    (cd ${path}/build_${board}; cp ${app_target}.xe ${DIST_DIR}/${name}.xe)
    if [ "${data_partition_target}" != "NONE" ]; then
        if [ ! -f ${DIST_DIR}/${data_partition_target}_data_partition.bin ]; then
            # need to make the data partition file for the data_partition_target
            #  this is generated once per data_partition_target and later copied
            #  to match the name of the app_target.
            echo '======================================================'
            echo '= Making data partition for' ${data_partition_target}
            echo '======================================================'
            (cd ${path}/build_${board}; log_errors $CI_BUILD_TOOL make_data_partition_${data_partition_target} $CI_BUILD_TOOL_ARGS)
            (cd ${path}/build_${board}; cp ${data_partition_target}_data_partition.bin ${DIST_DIR})
        fi
    fi
done
