#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XTC_DIR="${XMOS_TOOL_PATH:-/Applications/XMOS_XTC_15.3.1}"
CMAKE_BIN="${CMAKE_BIN:-/opt/homebrew/opt/cmake/bin/cmake}"
NINJA_BIN="${NINJA_BIN:-/opt/homebrew/opt/ninja/bin/ninja}"
PYTHON_BIN="${PYTHON_BIN:-/opt/homebrew/bin/python3.10}"
VENV_DIR="${ROOT_DIR}/.venv"
BUILD_DIR="${ROOT_DIR}/build"
TARGET="satellite1_firmware_fixed_delay"

if [[ ! -x "${XTC_DIR}/bin/xcc" ]]; then
  echo "XTC Tools not found at ${XTC_DIR}."
  echo "Install XMOS XTC Tools 15.3.1, or set XMOS_TOOL_PATH to its install directory."
  exit 1
fi

if [[ ! -x "${CMAKE_BIN}" ]]; then
  echo "CMake not found at ${CMAKE_BIN}."
  exit 1
fi

if [[ ! -x "${NINJA_BIN}" ]]; then
  echo "Ninja not found at ${NINJA_BIN}."
  exit 1
fi

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  "${PYTHON_BIN}" -m venv "${VENV_DIR}"
fi

"${VENV_DIR}/bin/pip" install -r "${ROOT_DIR}/requirements.txt"

export XMOS_TOOL_PATH="${XTC_DIR}"
export XMOS_HOME="${XMOS_HOME:-${HOME}/.xmos}"
export PATH="${VENV_DIR}/bin:${XTC_DIR}/bin:${XTC_DIR}/libexec:${PATH}"
export LD_LIBRARY_PATH="${XTC_DIR}/lib:${LD_LIBRARY_PATH:-}"
export DYLD_LIBRARY_PATH="${XTC_DIR}/lib:${DYLD_LIBRARY_PATH:-}"
export XCC_C_INCLUDE_PATH="${XTC_DIR}/target/include:${XTC_DIR}/target/include/clang"
export XCC_XC_INCLUDE_PATH="${XTC_DIR}/target/include/xc:${XCC_C_INCLUDE_PATH}"
export XCC_CPLUS_INCLUDE_PATH="${XCC_C_INCLUDE_PATH}:${XTC_DIR}/target/include/c++/v1"
export XCC_ASSEMBLER_INCLUDE_PATH="${XCC_C_INCLUDE_PATH}"
export XCC_LIBRARY_PATH="${XTC_DIR}/target/lib"
export XCC_DEVICE_PATH="${XTC_DIR}/configs:${XTC_DIR}/configs/.deprecated"
export XCC_TARGET_PATH="${XMOS_HOME}/targets:${XTC_DIR}/targets:${XTC_DIR}/targets/.deprecated"
export XCC_EXEC_PREFIX="${XTC_DIR}/libexec/"
export XMOS_DOC_PATH="${XTC_DIR}/doc"
export PYTHON_HOME="${XTC_DIR}/lib/jython"
export PYTHONPATH="${PYTHONPATH:-}:${XTC_DIR}/lib/python"
export PYTHON_VERBOSE="warning"
export XMOS_CACHE_PATH="${XMOS_HOME}/cache"
export XMOS_REPO_PATH="${XMOS_HOME}/repos"
export XMOS_MAKE_PATH="${XTC_DIR}/build"
export XMOS_CMAKE_PATH="${XTC_DIR}/build/xcommon_cmake"

"${CMAKE_BIN}" \
  -G Ninja \
  -B "${BUILD_DIR}" \
  -S "${ROOT_DIR}" \
  --toolchain "${ROOT_DIR}/xmos_cmake_toolchain/xs3a.cmake" \
  -DCMAKE_MAKE_PROGRAM="${NINJA_BIN}" \
  -DPython3_EXECUTABLE="${VENV_DIR}/bin/python"

"${CMAKE_BIN}" --build "${BUILD_DIR}" --target "create_flash_img_${TARGET}"

echo "Built ${BUILD_DIR}/${TARGET}.factory.bin"
echo "MD5  ${BUILD_DIR}/${TARGET}.factory.md5"
