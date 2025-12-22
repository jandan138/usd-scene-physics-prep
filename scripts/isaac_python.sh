#!/usr/bin/env bash
set -euo pipefail

# A portable wrapper to run Python scripts inside Omniverse Isaac Sim's Python.
# It tries to locate the Isaac Sim installation automatically, or uses
# ISAAC_SIM_ROOT if provided.
#
# Usage:
#   ./scripts/isaac_python.sh <script.py> [args...]
#
# Environment variables:
#   ISAAC_SIM_ROOT  Absolute path to Isaac Sim install directory that contains python.sh
#

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <script.py> [args...]" >&2
  exit 2
fi

resolve_isaac_root() {
  # 1) Respect explicit env var
  if [[ -n "${ISAAC_SIM_ROOT:-}" && -x "${ISAAC_SIM_ROOT}/python.sh" ]]; then
    echo "$ISAAC_SIM_ROOT"
    return 0
  fi
  # 2) Common path inside official Docker container
  if [[ -x "/isaac-sim/python.sh" ]]; then
    echo "/isaac-sim"
    return 0
  fi
  # 3) User-local OV packages (default for Isaac Sim standalone installs)
  local ov_root="${HOME}/.local/share/ov/pkg"
  if [[ -d "$ov_root" ]]; then
    local candidates=()
    while IFS= read -r -d '' dir; do
      candidates+=("$dir")
    done < <(find "$ov_root" -maxdepth 1 -type d -name 'isaac_sim-*' -print0 2>/dev/null)
    if [[ ${#candidates[@]} -gt 0 ]]; then
      IFS=$'\n' read -r -d '' -a sorted < <(printf '%s\n' "${candidates[@]}" | sort -V && printf '\0') || true
      for (( idx=${#sorted[@]}-1; idx>=0; idx--)); do
        local dir="${sorted[$idx]}"
        [[ -z "$dir" ]] && continue
        if [[ -x "$dir/python.sh" ]]; then
          echo "$dir"
          return 0
        fi
      done
    fi
  fi
  # 4) A few common system locations
  for base in /opt/nvidia/isaac-sim /opt/NVIDIA/isaac-sim /opt/omniverse/isaac-sim; do
    if [[ -x "${base}/python.sh" ]]; then
      echo "$base"
      return 0
    fi
  done
  return 1
}

ISAAC_ROOT="$(resolve_isaac_root)" || {
  echo "ERROR: Could not locate Isaac Sim installation (python.sh not found)." >&2
  echo "Hint: export ISAAC_SIM_ROOT=\"/abs/path/to/isaac_sim-<version>\"" >&2
  echo "       and re-run: ISAAC_SIM_ROOT=... $0 <script.py> [args...]" >&2
  exit 1
}

RUNNER="${ISAAC_ROOT}/python.sh"

# Optional environment preparation similar to /isaac-sim/isaac_python.sh
# 1) Prepend all directories that contain a 'pxr' package under extscache to PYTHONPATH
EXTSCACHE_DIR="${ISAAC_ROOT}/extscache"
if [[ -d "$EXTSCACHE_DIR" ]]; then
  while IFS= read -r -d '' pxrdir; do
    parent_dir="$(dirname "$pxrdir")"
    if [[ -z "${PYTHONPATH:-}" ]]; then
      export PYTHONPATH="${parent_dir}"
    else
      export PYTHONPATH="${parent_dir}:${PYTHONPATH}"
    fi
  done < <(find "$EXTSCACHE_DIR" -maxdepth 4 -type d -name pxr -print0 2>/dev/null || true)

  # 2) Locate USD shared libs (e.g., omni.usd.libs/bin/libtf.so) and extend LD_LIBRARY_PATH
  USD_LIB_SO="$(find "$EXTSCACHE_DIR" -maxdepth 6 -type f -name 'libtf.so' 2>/dev/null | grep -m1 'omni\.usd\.libs' || true)"
  if [[ -n "$USD_LIB_SO" ]]; then
    USD_LIB_DIR="$(dirname "$USD_LIB_SO")"
  else
    USD_LIB_DIR=""
  fi
  add_ld=()
  [[ -n "$USD_LIB_DIR" && -d "$USD_LIB_DIR" ]] && add_ld+=("$USD_LIB_DIR")
  [[ -d "${ISAAC_ROOT}/kit/lib" ]] && add_ld+=("${ISAAC_ROOT}/kit/lib")
  [[ -d "${ISAAC_ROOT}/kit/plugins" ]] && add_ld+=("${ISAAC_ROOT}/kit/plugins")
  if [[ ${#add_ld[@]} -gt 0 ]]; then
    joined="$(IFS=":"; echo "${add_ld[*]}")"
    if [[ -z "${LD_LIBRARY_PATH:-}" ]]; then
      export LD_LIBRARY_PATH="$joined"
    else
      export LD_LIBRARY_PATH="$joined:${LD_LIBRARY_PATH}"
    fi
  fi
fi

exec "$RUNNER" "$@"
