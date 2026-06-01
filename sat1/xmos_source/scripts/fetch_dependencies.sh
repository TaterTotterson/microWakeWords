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

while read -r path url commit; do
  [[ -z "${path}" || "${path}" == \#* ]] && continue
  clone_checkout "${path}" "${url}" "${commit}"
done < "${LOCK_FILE}"
