#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace_dir}/tools/workspace_env.sh"
if [[ ! -d "${COMPETITION_LOCAL_DIR}" ]]; then
    (umask 077; mkdir -p "${COMPETITION_LOCAL_DIR}")
fi
ardupilot_dir="${COMPETITION_LOCAL_DIR}/dependencies/ardupilot"
ardupilot_commit="b72075518ff1409438a817a1dd9eea4828d28642"
ardupilot_patch="${workspace_dir}/tools/patches/ardupilot-sitl-initialization.patch"
ardupilot_optflow_patch="${workspace_dir}/tools/patches/ardupilot-optical-flow-rad.patch"
ros_keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
gazebo_keyring="/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"

log_step() {
    echo
    echo "========== $1 =========="
}

if [[ "$(id -u)" -eq 0 ]]; then
    echo "请使用普通用户运行本脚本；脚本会在需要时调用 sudo。" >&2
    exit 1
fi

if [[ ! -r /etc/os-release ]]; then
    echo "无法识别操作系统，仅支持 Ubuntu 22.04。" >&2
    exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "当前系统为 ${PRETTY_NAME:-未知系统}，本脚本仅支持 Ubuntu 22.04。" >&2
    exit 1
fi

if [[ "$(uname -m)" != x86_64 ]]; then
    echo '目前支持 Ubuntu 22.04 amd64；其他架构尚未验证。' >&2
    exit 1
fi
if [[ -n "${VIRTUAL_ENV:-}${CONDA_PREFIX:-}" ]]; then
    echo '请退出 Python 虚拟环境或 Conda 后安装；ROS Humble 使用系统 Python 3.10。' >&2
    exit 1
fi

sudo -v

log_step "安装基础工具"
sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ca-certificates locales software-properties-common curl gnupg lsb-release git \
    build-essential cmake ccache g++ gawk make libtool libtool-bin \
    libxml2-dev libxslt1-dev ppp \
    python3-pip python3-venv python3-dev python3-setuptools python3-numpy \
    python3-opencv python3-pyparsing python3-psutil pkg-config

sudo locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

log_step "配置并安装 ROS 2 Humble"
sudo add-apt-repository -y universe
sudo curl -fsSL --retry 3 \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o "${ros_keyring}"

architecture="$(dpkg --print-architecture)"
ubuntu_codename="${UBUNTU_CODENAME:-jammy}"
echo "deb [arch=${architecture} signed-by=${ros_keyring}] ${ROS_APT_URL:-http://packages.ros.org/ros2/ubuntu} ${ubuntu_codename} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-desktop ros-dev-tools python3-argcomplete \
    python3-colcon-common-extensions python3-rosdep python3-vcstool

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

log_step "配置并安装 Gazebo Sim 7"
sudo curl -fsSL --retry 3 https://packages.osrfoundation.org/gazebo.gpg \
    -o "${gazebo_keyring}"
echo "deb [arch=${architecture} signed-by=${gazebo_keyring}] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${ubuntu_codename} main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    gz-garden ros-humble-ros-gzgarden libgz-transport12-dev libgz-msgs9-dev \
    python3-pyqt5

log_step "准备固定版本的 ArduPilot"
new_checkout=false
if [[ ! -e "${ardupilot_dir}/.git" ]]; then
    mkdir -p "$(dirname "${ardupilot_dir}")"
    git clone --filter=blob:none --no-checkout --depth 1 \
        https://github.com/ArduPilot/ardupilot.git "${ardupilot_dir}"
    new_checkout=true
fi

current_commit="$(git -C "${ardupilot_dir}" rev-parse HEAD)"
if [[ "${current_commit}" != "${ardupilot_commit}" || "${new_checkout}" == true ]]; then
    if [[ "${new_checkout}" == false && -n "$(git -C "${ardupilot_dir}" status --porcelain)" ]]; then
        echo "ArduPilot 目录存在本地修改，拒绝切换固定版本：" >&2
        git -C "${ardupilot_dir}" status --short >&2
        exit 1
    fi
    git -C "${ardupilot_dir}" fetch --depth 1 origin "${ardupilot_commit}"
    git -C "${ardupilot_dir}" checkout --detach "${ardupilot_commit}"
fi
git -C "${ardupilot_dir}" submodule update --init --recursive --depth 1

apply_ardupilot_patch() {
    local patch_file="$1"
    if git -C "${ardupilot_dir}" apply --reverse --check "${patch_file}" \
        >/dev/null 2>&1; then
        echo "ArduPilot 补丁已应用：$(basename "${patch_file}")"
    elif git -C "${ardupilot_dir}" apply --check "${patch_file}"; then
        git -C "${ardupilot_dir}" apply "${patch_file}"
    else
        echo "ArduPilot 补丁无法安全应用：${patch_file}" >&2
        exit 1
    fi
}

apply_ardupilot_patch "${ardupilot_patch}"
apply_ardupilot_patch "${ardupilot_optflow_patch}"

log_step "安装 Python 依赖并编译 ArduPilot"
python3 -m pip install --user -r "${workspace_dir}/requirements-dev.txt" \
    -r "${workspace_dir}/requirements-sitl.txt"
(
    cd "${ardupilot_dir}"
    ./waf configure --board sitl
    ./waf copter -j "${BUILD_JOBS:-2}"
)
test -x "${ardupilot_dir}/build/sitl/bin/arducopter"

log_step "安装 ROS 依赖并编译比赛仿真"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

cd "${workspace_dir}"
rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys 'gz-transport12 gz-msgs9 ros_gz_bridge ros_gz_interfaces'
bash "${workspace_dir}/tools/build_workspace.sh"
bash "${workspace_dir}/start.sh" --headless --check

log_step "部署完成"
echo "请在新终端执行："
echo "cd ${workspace_dir}"
echo "./start.sh --check"
echo "./start.sh"
