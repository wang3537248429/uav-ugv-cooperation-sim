#!/usr/bin/env bash
# Usage: source tools/activate.sh
source "$(dirname "${BASH_SOURCE[0]}")/workspace_env.sh" || return 1
if [[ ! -f "${COMPETITION_LOCAL_DIR}/install/setup.bash" ]]; then
    echo 'Build the simulation first: make build' >&2
    return 1
fi
if [[ ! -f "${COMPETITION_LOCAL_DIR}/install/.aeromind-source" ]] ||
   [[ "$(cat "${COMPETITION_LOCAL_DIR}/install/.aeromind-source")" != "${workspace_dir}" ]]; then
    echo 'Build belongs to another source location. Run make build in this repository.' >&2
    return 1
fi
competition_restore_nounset=false
if [[ "$-" == *u* ]]; then competition_restore_nounset=true; set +u; fi
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${COMPETITION_LOCAL_DIR}/install/setup.bash"
if [[ "${competition_restore_nounset}" == true ]]; then set -u; fi
unset competition_restore_nounset
