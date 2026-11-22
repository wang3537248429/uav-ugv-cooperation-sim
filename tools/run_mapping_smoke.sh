#!/usr/bin/env bash
# Two fresh physics sessions: build a map, then navigate using the frozen map.
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace_dir}/tools/workspace_env.sh"
mkdir -p "${COMPETITION_LOCAL_DIR}/artifacts"
result_dir="$(mktemp -d "${COMPETITION_LOCAL_DIR}/artifacts/navigation-smoke-XXXXXX")"
map_file="${result_dir}/arena.npz"
echo "Map-navigation regression directory: ${result_dir}"
timeout_seconds="${1:-900}"
if [[ $# -gt 0 ]]; then shift; fi
common_args=("$@")

SMOKE_LOG_FILE="${result_dir}/mapping-launch.log" \
  bash "${workspace_dir}/tools/run_headless_smoke.sh" "${timeout_seconds}" \
    external_nav_source:=mapping "map_file:=${map_file}" "${common_args[@]}" \
  | tee "${result_dir}/mapping-result.log"
test -s "${map_file}"
test -s "${result_dir}/arena.pcd"
map_hash="$(sha256sum "${map_file}")"
SMOKE_LOG_FILE="${result_dir}/localization-launch.log" \
  bash "${workspace_dir}/tools/run_headless_smoke.sh" "${timeout_seconds}" \
    external_nav_source:=localization "map_file:=${map_file}" "${common_args[@]}" \
  | tee "${result_dir}/localization-result.log"
test "${map_hash}" = "$(sha256sum "${map_file}")"
echo "Mapping, saved-map navigation and read-only map checks passed: ${result_dir}"
