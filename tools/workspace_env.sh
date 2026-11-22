#!/usr/bin/env bash
# Source this file from any directory; generated data lives outside the source tree.
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export COMPETITION_SOURCE_DIR="${workspace_dir}"
export COMPETITION_LOCAL_DIR="$(realpath -m -- "${COMPETITION_LOCAL_DIR:-${XDG_CACHE_HOME:-${HOME}/.cache}/aeromind-sim}")"
case "${COMPETITION_LOCAL_DIR}/" in
    "${workspace_dir}/"*)
        echo 'COMPETITION_LOCAL_DIR must be outside the public source repository.' >&2
        return 1 2>/dev/null || exit 1
        ;;
esac
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="${COMPETITION_LOCAL_DIR}/logs/ros"
export SITL_WORKDIR="${SITL_WORKDIR:-${COMPETITION_LOCAL_DIR}/sitl}"
export ARDUCOPTER_BIN="${ARDUCOPTER_BIN:-${COMPETITION_LOCAL_DIR}/dependencies/ardupilot/build/sitl/bin/arducopter}"
