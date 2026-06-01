#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK_FILE="${ROOT_DIR}/submodules.lock"

clone_checkout() {
  local path="$1"
  local url="$2"
  local commit="$3"
  local dest="${ROOT_DIR}/${path}"

  if [[ -d "${dest}/.git" ]]; then
    git -C "${dest}" fetch --tags --force
  elif [[ -e "${dest}" ]]; then
    echo "Refusing to replace existing non-git path: ${dest}" >&2
    echo "Move it aside, then rerun this script." >&2
    exit 1
  else
    mkdir -p "$(dirname "${dest}")"
    git clone "${url}" "${dest}"
  fi

  git -C "${dest}" checkout --detach "${commit}"
  git -C "${dest}" submodule update --init --recursive
}

apply_patch_if_needed() {
  local path="$1"
  local patch="$2"
  local dest="${ROOT_DIR}/${path}"
  local patch_file="${ROOT_DIR}/${patch}"

  if git -C "${dest}" apply --check "${patch_file}" >/dev/null 2>&1; then
    git -C "${dest}" apply "${patch_file}"
  else
    echo "Skipping patch, already applied or not applicable: ${patch}" >&2
  fi
}

while read -r path url commit; do
  [[ -z "${path}" || "${path}" == \#* ]] && continue
  clone_checkout "${path}" "${url}" "${commit}"
done < "${LOCK_FILE}"

apply_patch_if_needed "modules/rtos" "patches/rtos-dual-issue-entersp.patch"
apply_patch_if_needed "modules/io/modules/mic_array" "patches/mic-array-non-mic-tile-stub.patch"
