#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/workspace_env.sh"
if [[ ! -d "${COMPETITION_LOCAL_DIR}" ]]; then
    (umask 077; mkdir -p "${COMPETITION_LOCAL_DIR}")
fi
set +u
# Do not record overlays from an old/moved workspace in the generated setup chain.
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
cd "${workspace_dir}"
rm -f "${COMPETITION_LOCAL_DIR}/install/.aeromind-source"
colcon --log-base "${COMPETITION_LOCAL_DIR}/logs/colcon" build \
    --symlink-install --cmake-clean-cache --base-paths "${workspace_dir}/src" \
    --build-base "${COMPETITION_LOCAL_DIR}/build" \
    --install-base "${COMPETITION_LOCAL_DIR}/install" \
    --parallel-workers "${BUILD_JOBS:-2}" "$@"
printf '%s\n' "${workspace_dir}" >"${COMPETITION_LOCAL_DIR}/install/.aeromind-source"
