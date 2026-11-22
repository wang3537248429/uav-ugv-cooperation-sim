#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace_dir}/tools/workspace_env.sh"
# The complete route includes a conservative post-strike return and guided
# landing. Keep enough wall-clock margin for slower CI runners.
timeout_seconds="${1:-600}"
if [[ $# -gt 0 ]]; then
  shift
fi
launch_args=("$@")
navigation_selected=false
for argument in "${launch_args[@]}"; do
  if [[ "${argument}" == external_nav_source:=* ]]; then navigation_selected=true; fi
done
if [[ "${navigation_selected}" == false ]]; then
  launch_args+=(external_nav_source:=truth)
fi
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
workspace_setup="${COMPETITION_LOCAL_DIR}/install/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS setup not found: ${ros_setup}" >&2
  exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install" >&2
  exit 1
fi

set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

mkdir -p "${COMPETITION_LOCAL_DIR}/artifacts"
log_file="${SMOKE_LOG_FILE:-${COMPETITION_LOCAL_DIR}/artifacts/headless-smoke.log}"
monitor_args=(--check-ugv)
for argument in "${launch_args[@]}"; do
  case "${argument}" in
    external_nav_source:=mapping) monitor_args+=(--navigation-mode mapping) ;;
    external_nav_source:=localization) monitor_args+=(--navigation-mode localization) ;;
  esac
done

# Keep the setsid supervisor alive while the new session starts.  Without
# --wait, util-linux may fork and return the helper PID immediately; the
# monitor then mistakes that normal fork for a crashed launch and the child
# ROS graph is left holding the simulation lock.
display_args=(--headless)
if [[ "${SMOKE_VISUAL:-0}" == 1 ]]; then display_args=(); fi
setsid --wait bash "${workspace_dir}/start.sh" \
  "${display_args[@]}" auto_start:=true "${launch_args[@]}" >"${log_file}" 2>&1 &
launch_pid=$!

# The --wait supervisor and the actual launch have different process groups.
# Record the latter so cleanup can terminate both groups on every exit path.
launch_group_pid=''
for _ in {1..50}; do
  child_pid="$(pgrep -P "${launch_pid}" | head -n 1 || true)"
  if [[ -n "${child_pid}" ]]; then
    launch_group_pid="$(ps -o pgid= -p "${child_pid}" | tr -d ' ' || true)"
    [[ -n "${launch_group_pid}" ]] && break
  fi
  sleep 0.1
done

cleanup() {
  for process_group in "${launch_group_pid}" "${launch_pid}"; do
    if [[ "${process_group}" =~ ^[0-9]+$ ]] && (( process_group > 1 )); then
      kill -TERM -- "-${process_group}" 2>/dev/null || true
    fi
  done
  for _ in {1..50}; do
    supervisor_alive=false
    launch_group_alive=false
    kill -0 "${launch_pid}" 2>/dev/null && supervisor_alive=true
    if [[ "${launch_group_pid}" =~ ^[0-9]+$ ]]; then
      kill -0 -- "-${launch_group_pid}" 2>/dev/null && launch_group_alive=true
    fi
    if [[ "${supervisor_alive}" == false && "${launch_group_alive}" == false ]]; then
      break
    fi
    sleep 0.1
  done
  for process_group in "${launch_group_pid}" "${launch_pid}"; do
    if [[ "${process_group}" =~ ^[0-9]+$ ]] && (( process_group > 1 )); then
      kill -KILL -- "-${process_group}" 2>/dev/null || true
    fi
  done
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
trap 'exit 129' HUP

set +e
python3 "${workspace_dir}/tools/mission_smoke_monitor.py" \
  --timeout "${timeout_seconds}" --launch-pid "${launch_pid}" "${monitor_args[@]}"
monitor_status=$?
set -e

cleanup
trap - EXIT INT TERM

if [[ "${monitor_status}" -ne 0 ]]; then
  echo "Smoke test failed. Last mission events:" >&2
  grep -E "状态转换|ABORT|COMPLETE|Traceback|ERROR|FAILSAFE" \
    "${log_file}" | tail -80 >&2 || true
  echo "Full log: ${log_file}" >&2
  exit "${monitor_status}"
fi

if grep -Eqi \
  "Traceback|FAILSAFE|状态转换: .* → ABORT|Internal Errors|process has died" \
  "${log_file}"; then
  echo "Mission reached COMPLETE but the log contains a fatal marker." >&2
  grep -Ei \
    "Traceback|FAILSAFE|状态转换: .* → ABORT|Internal Errors|process has died" \
    "${log_file}" | tail -40 >&2 || true
  echo "Full log: ${log_file}" >&2
  exit 4
fi

echo "Competition smoke test passed (SMOKE_VISUAL=${SMOKE_VISUAL:-0})."
echo "Log: ${log_file}"
