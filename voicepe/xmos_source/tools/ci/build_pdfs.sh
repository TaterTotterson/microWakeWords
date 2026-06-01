#!/bin/bash
set -e

# help text
help()
{
   echo "XCORE-VOICE module PDF builder script"
   echo
   echo "Syntax: build_pdfs.sh"
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

XCORE_VOICE_ROOT=`git rev-parse --show-toplevel`
DOC_BUILDER_IMAGE=ghcr.io/xmos/xmosdoc

source ${XCORE_VOICE_ROOT}/tools/ci/helper_functions.sh

# setup distribution folder
DIST_DIR=${XCORE_VOICE_ROOT}/dist_pdfs
mkdir -p ${DIST_DIR}

# setup configurations
# row format is: "module_path  generate exclude_patterns generated_filename   final_filename  title"
standard_modules=(
    "modules/io/modules/mic_array   yes programming_guide.pdf   doc_excludes.txt   lib_mic_array_programming_guide.pdf lib\_mic\_array" 
    "modules/io   yes programming_guide.pdf   exclude_patterns.inc   fwk_io_programming_guide.pdf fwk\_io"
    "modules/voice  yes user_guide.pdf   exclude_patterns.inc   fwk_voice_programming_guide.pdf fwk\_voice"
    "modules/rtos   yes programming_guide.pdf   exclude_patterns.inc   rtos_programming_guide.pdf RTOS"
    "modules/rtos   yes build_system_guide.pdf   exclude_patterns.inc   build_system_user_guide.pdf"
)

# *****************************************************************
# NOTE: some modules are not standard and are built individually
# *****************************************************************

# perform builds on standard modules
for ((i = 0; i < ${#standard_modules[@]}; i += 1)); do
    read -ra FIELDS <<< ${standard_modules[i]}
    rel_path="${FIELDS[0]}"
    gen_flag="${FIELDS[1]}"
    gen_name="${FIELDS[2]}"
    expat_file="${FIELDS[3]}"
    fin_name="${FIELDS[4]}"
    title="${FIELDS[5]}"
    full_path="${XCORE_VOICE_ROOT}/${rel_path}"

    if [[ ${gen_flag} = "yes" ]] ; then
        echo '******************************************************'
        echo '* Building PDFs for' ${rel_path}
        echo '******************************************************'

        # build docs
        (cd ${full_path}; docker run --rm -t -u "$(id -u):$(id -g)" -v $(pwd):/build ${DOC_BUILDER_IMAGE})
    fi

    # copy to dist folder
    (cd ${full_path}/doc/_build/pdf; cp ${gen_name} ${DIST_DIR}/${fin_name})
done

# perform builds on non-standard modules

echo '******************************************************'
echo '* Building PDFs for lib_xcore_math'
echo '******************************************************'
# lib_xcore_math is non standard because the doc_builder returns a non-zero return code but does not generate an error.  
#  To workaround this, the call to docker is redirected so the return code can be ignored.  
full_path="${XCORE_VOICE_ROOT}/modules/core/modules/xcore_math/lib_xcore_math"

# build docs
(cd ${full_path}; docker run --rm -t -u "$(id -u):$(id -g)" -v $(pwd):/build ${DOC_BUILDER_IMAGE} || echo "Container always falsely reports an error. Ignoring error.")

# copy to dist folder
(cd ${full_path}/doc/_build/pdf; cp programming_guide.pdf ${DIST_DIR}/lib_xcore_math_programming_guide.pdf)
