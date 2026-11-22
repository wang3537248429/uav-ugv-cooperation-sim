#!/usr/bin/env bash
# User-facing entry point; the ROS launch file owns simulation composition.
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${workspace_dir}/tools/workspace_env.sh"
build_first=false
check_only=false
launch_args=()

usage() {
    cat <<'EOF'
用法：./start.sh [选项] [名称:=值 ...]

默认启动点云导航：没有地图时建图，已有地图时加载定位，并自动开始仿真任务。

选项：
  --headless       不打开 Gazebo 和 RViz 窗口
  --optical-flow   使用仿真光流里程计作为 ExternalNav
  --truth          使用真值定位基线（仅作对照）
  --mapping        使用三维雷达点云在线建图并导航（首次建图）
  --localize       加载已保存地图，用雷达匹配定位并导航
  --map PATH       地图路径，默认运行缓存目录中的 maps/arena.npz
  --manual         等待手动发送 /mission/start
  --gcs PORT       将 MAVLink 转发到本机 UDP PORT（默认 14550），供 Mission Planner/QGC 连接
  --no-rviz        不打开 RViz
  --no-camera      不打开独立相机窗口
  --software-render  使用软件渲染（仅硬件 OpenGL 不可用时）
  --build          先编译 ROS 工作空间
  --check          检查环境与 launch 文件，不启动仿真
  -h, --help       显示帮助

示例：
  ./start.sh
  ./start.sh --headless --optical-flow
  ./start.sh --mapping
  ./start.sh --localize
  ./start.sh --build --check
  ./start.sh --manual gz_partition:=competition_preview

更多说明：README.md
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --headless) launch_args+=(gz_headless:=true rviz:=false) ;;
        --optical-flow)
            launch_args+=(optical_flow_odom:=true external_nav_source:=odometry)
            ;;
        --mapping) launch_args+=(external_nav_source:=mapping) ;;
        --localize) launch_args+=(external_nav_source:=localization) ;;
        --truth) launch_args+=(external_nav_source:=truth) ;;
        --map)
            [[ $# -ge 2 ]] || { echo '--map 需要地图路径' >&2; exit 2; }
            launch_args+=("map_file:=$2")
            shift
            ;;
        --manual) launch_args+=(auto_start:=false) ;;
        --gcs)
            [[ $# -ge 2 ]] || { echo '--gcs 需要 UDP 端口' >&2; exit 2; }
            launch_args+=("gcs_output_url:=udpout:127.0.0.1:$2")
            shift
            ;;
        --no-rviz) launch_args+=(rviz:=false) ;;
        --no-camera) launch_args+=(camera_view:=false) ;;
        --software-render) export LIBGL_ALWAYS_SOFTWARE=1 ;;
        --build) build_first=true ;;
        --check) check_only=true ;;
        --) shift; launch_args+=("$@"); break ;;
        *:=*) launch_args+=("$1") ;;
        *) echo "无法识别参数：$1；使用 --help 查看用法。" >&2; exit 2 ;;
    esac
    shift
done

cd "${workspace_dir}"
# Gazebo/RViz use XWayland reliably on GNOME Wayland; retain explicit overrides.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
selected_navigation=''
selected_map="${COMPETITION_LOCAL_DIR}/maps/arena.npz"
for argument in "${launch_args[@]}"; do
    case "${argument}" in
        external_nav_source:=*) selected_navigation="${argument#*:=}" ;;
        map_file:=*) selected_map="${argument#*:=}" ;;
    esac
done
if [[ -z "${selected_navigation}" ]]; then
    selected_map="$(python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).expanduser().absolute())' "${selected_map}")"
    if [[ -f "${selected_map}" ]]; then
        selected_navigation=localization
    else
        selected_navigation=mapping
    fi
    launch_args+=("external_nav_source:=${selected_navigation}" "map_file:=${selected_map}")
fi
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
workspace_setup="${COMPETITION_LOCAL_DIR}/install/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
    echo "未找到 ROS 环境：${ros_setup}。Ubuntu 22.04 首次安装请运行 make setup。" >&2
    exit 1
fi

# ROS setup scripts can reference unset variables.
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

if [[ "${build_first}" == true ]]; then
    command -v colcon >/dev/null || { echo "未找到 colcon，请先运行 make setup。" >&2; exit 1; }
    bash "${workspace_dir}/tools/build_workspace.sh"
fi

if [[ ! -f "${workspace_setup}" ]]; then
    echo "工作空间尚未编译。请运行 make build，或使用 ./start.sh --build。" >&2
    exit 1
fi
if [[ ! -f "${COMPETITION_LOCAL_DIR}/install/.aeromind-source" ]] ||
   [[ "$(cat "${COMPETITION_LOCAL_DIR}/install/.aeromind-source")" != "${workspace_dir}" ]]; then
    echo '构建目录来自另一个源码位置或旧版构建。请在当前仓库运行 make build。' >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "${workspace_setup}"
set -u

for executable in ros2 gz; do
    if ! command -v "${executable}" >/dev/null; then
        echo "未找到 ${executable}，请先完成 make setup。" >&2
        exit 1
    fi
done

python3 "${workspace_dir}/tools/check_runtime.py"

# Match the launch file's executable selection, including explicit overrides.
start_sitl=true
arducopter_bin="${ARDUCOPTER_BIN}"
for argument in "${launch_args[@]}"; do
    case "${argument}" in
        start_sitl:=*) start_sitl="${argument#*:=}" ;;
        arducopter_bin:=*) arducopter_bin="${argument#*:=}" ;;
    esac
done
case "${start_sitl,,}" in
    false|0) ;;
    *)
        if [[ ! -x "${arducopter_bin}" ]]; then
            echo "未找到可执行的 ArduCopter SITL：${arducopter_bin}" >&2
            echo "首次安装请运行 make setup；已有编译版本可通过 ARDUCOPTER_BIN 指定。" >&2
            exit 1
        fi
        if [[ "${arducopter_bin}" != /* ]]; then
            arducopter_bin="${workspace_dir}/${arducopter_bin}"
        fi
        launch_args+=("arducopter_bin:=${arducopter_bin}")
        ;;
esac

if [[ "${check_only}" == true ]]; then
    ros2 launch --show-args ap_simulation sim_all.launch.py "${launch_args[@]}"
    printf '\n环境与启动文件检查通过。启动命令：'
    printf '%q ' ros2 launch ap_simulation sim_all.launch.py "${launch_args[@]}"
    printf '\n'
    exit 0
fi

# The stack uses fixed SITL/JSON ports. Protect all checkouts on this host,
# including callers choosing a different ROS domain or Gazebo partition.
runtime_dir="${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}/competition_ws-$(id -u)"
mkdir -p "${runtime_dir}"
exec 9>"${runtime_dir}/simulation.lock"
if ! flock -n 9; then
    echo '已有本项目仿真实例运行，请先停止它，再启动建图或地图定位。' >&2
    exit 1
fi
python3 - "${start_sitl}" <<'PY'
import socket
import sys

ports = [(socket.SOCK_DGRAM, 9002)]
if sys.argv[1].lower() not in ('false', '0'):
    ports.append((socket.SOCK_STREAM, 5760))
for kind, port in ports:
    with socket.socket(socket.AF_INET, kind) as connection:
        if kind == socket.SOCK_STREAM:
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            connection.bind(('127.0.0.1', port))
        except OSError as error:
            sys.exit(f'仿真端口 {port} 不可用：{error}。请先停止占用该端口的实例。')
PY

echo "启动空地协同比赛仿真；在当前终端按 Ctrl+C 停止。"
exec ros2 launch ap_simulation sim_all.launch.py "${launch_args[@]}"
