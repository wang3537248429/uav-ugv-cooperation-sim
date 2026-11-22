# 开发环境与维护流程

本文说明如何在不污染源码仓库的前提下修改本项目的代码。普通用户先阅读[运行手册](运行手册.md)；需要修改 ROS 节点、模型或测试时，再按本文准备开发环境。

## 工具链

项目的正式验证环境为 Ubuntu 22.04 amd64、ROS 2 Humble、Gazebo Sim 7 和 Python 3.10。安装脚本会安装 `ros-humble-desktop`、`ros-dev-tools`、`colcon`、Gazebo Garden 兼容包、OpenCV、NumPy、编译工具和 ArduPilot SITL。CI 另外覆盖 Python 3.11 的独立测试；ROS 构建仍使用 Humble。

源码树不保存 Python 虚拟环境、ROS `build/install/log`、ArduPilot checkout 或地图。它们通过 `COMPETITION_LOCAL_DIR` 写到仓库外。独立 Python 工具可以使用虚拟环境：

```bash
source tools/workspace_env.sh
python3 -m venv "$COMPETITION_LOCAL_DIR/venv"
source "$COMPETITION_LOCAL_DIR/venv/bin/activate"
python3 -m pip install -r requirements-dev.txt
```

运行 ROS 节点和 ROS 测试时，使用系统 Python 及 ROS 环境：

```bash
deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source tools/activate.sh
```

## 常用命令

依赖文件按用途拆分：

| 文件 | 用途 |
| --- | --- |
| `requirements.txt` | 仿真运行时的 Python 依赖 |
| `requirements-sitl.txt` | ArduPilot SITL 构建所需的 Python 依赖 |
| `requirements-test.txt` | ROS 和独立回归测试依赖 |
| `requirements-dev.txt` | lint、测试和发布检查工具 |

```bash
make lint          # flake8
make test          # tests/unit，不依赖 ROS
make test-ros      # tests/ros，需要 ROS 工作空间
make test-all      # 两组测试
make check         # lint、测试、场景和公开内容检查
make build         # colcon 构建
make model         # 从模型模板生成 competition_x500
make doctor        # 等同 ./start.sh --check
```

更改任务状态机、制导、时间戳、传感器、坐标转换或物理模型后，至少运行：

```bash
make check
make build
make test-all
make smoke
```

修改点云建图、地图格式或航点时，另外运行 `SMOKE_VISUAL=1 make smoke-mapping`，确认建图、重启、地图加载和定位阶段全部完成。修改光流基线时，还应运行 `make smoke SMOKE_ARGS='--optical-flow'`。

## 代码和文件约定

- 与 ROS 无关的数学、坐标和消息转换放在包内的纯 Python 模块，并为其添加 `tests/unit` 测试。
- 需要 ROS、Gazebo 或消息生命周期的行为放在节点模块，并在 `tests/ros` 增加回归测试。
- 默认参数放在所属包的配置或 `*_config.py`；本地覆盖文件使用 `*.local.yaml`，只放在私有目录。
- 修改 `src/ap_simulation/models/competition_x500/model.sdf.in` 后运行 `make model`，同时检查生成模型是否仍由模板唯一决定。
- ArduPilot 修改保存为 `tools/patches/*.patch`，记录上游版本和补丁用途，不把完整依赖树复制进仓库。
- 坐标转换必须在接口边界明确说明 NED、ENU、FLU 或 FRD，不能依赖变量名猜测。
- 不在源码中写入绝对路径、用户名、内网地址、设备序列号、令牌、密钥、运行日志或真实飞行数据。

## 参数覆盖

| 变量 | 默认值 | 作用 |
| --- | --- | --- |
| `COMPETITION_LOCAL_DIR` | `${XDG_CACHE_HOME:-$HOME/.cache}/aeromind-sim` | 所有构建、依赖和运行数据的根目录 |
| `SITL_WORKDIR` | `$COMPETITION_LOCAL_DIR/sitl` | ArduPilot 工作目录 |
| `ARDUCOPTER_BIN` | 私有目录下的 SITL 可执行文件 | 指定其他 ArduCopter 构建 |
| `ROS_DISTRO` | `humble` | ROS setup 和 ROS 测试目录 |
| `PYTHON` | `python3` | Makefile 使用的 Python 命令 |
| `BUILD_JOBS` | `2` | SITL 和 ROS 构建的并行数 |
| `SMOKE_TIMEOUT` | `600` | 每轮 smoke 超时秒数 |
| `SMOKE_VISUAL` | `0` | 设为 `1` 时启用可视化回归并自动清理进程 |
| `SMOKE_ARGS` | 空 | 追加到无头 smoke 脚本 |
| `RUN_ARGS` | 空 | 追加到 `start.sh` |

例如：

```bash
export COMPETITION_LOCAL_DIR=/srv/aeromind-sim-private
export ARDUCOPTER_BIN=/opt/ardupilot/build/sitl/bin/arducopter
make build
make test-all
```

## 提交前检查

不要直接提交整个工作目录。只暂存源码、测试、场景资源和文档，然后检查暂存内容：

```bash
git status --short
git diff --check
git add -A
git diff --cached --stat
python3 tools/check_open_source_hygiene.py --staged
make check
```

重点确认暂存内容没有用户主目录、主机名、私有目录、内网地址、`.env`、私钥、令牌、串口配置、日志、地图、视频或其他本机生成文件。发布源码包使用：

```bash
make release
```

输出位于 `$COMPETITION_LOCAL_DIR/releases/aeromind-sim-source.zip`，不包含 `.git` 和私有运行数据。
