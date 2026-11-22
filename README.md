# AeroMind Sim

AeroMind Sim 是一套基于 ROS 2、Gazebo Sim 和 ArduPilot SITL 的空地协同仿真工程。它提供室内无 GPS 的点云建图与定位、预设航线、相机识别、无人车路线，以及可选的 MAVLink 地面站连接。

仓库只包含源码、测试、场景资源和安装脚本。构建产物、地图、日志、SITL 依赖和截图统一写入仓库外的私有目录，避免把本机路径、运行数据或凭据带入 Git。

## 支持范围

当前验证环境为：

- Ubuntu 22.04 amd64
- ROS 2 Humble
- Gazebo Sim 7
- Python 3.10
- ArduPilot SITL（由 `make setup` 获取并编译）

安装前建议预留至少 20 GB 可用磁盘，供 ROS、Gazebo、SITL 源码和构建产物使用；构建 Docker 镜像还会保存镜像层。

建议使用支持 OpenGL 4.x 的显卡。没有可用硬件 OpenGL 时可以使用 `--software-render`，但 Gazebo 和 RViz 的帧率会降低。

这是仿真和算法验证工程，不是实机飞控配置，也不代表正式比赛或适航结论。场景中的相机、无人车和扎球过程仍有仿真简化。

## 快速开始

```bash
git clone REPOSITORY_URL aeromind-sim
cd aeromind-sim
bash tools/setup_ubuntu_22_04.sh
./start.sh --check
```

把 `REPOSITORY_URL` 替换成仓库地址。安装脚本需要联网和 `sudo`，会安装系统依赖、准备外部私有目录、编译固定提交的 ArduPilot SITL 和 ROS 工作空间，并运行依赖自检。已有 `make` 时，`make setup` 是同一个安装入口。请先退出 Conda / Python 虚拟环境。已有依赖时也可以直接执行 `make build`。

也可以下载源码 ZIP，解压后在源码根目录执行上述安装脚本。不需要作者的地图、缓存、依赖目录或本机配置。若解压工具没有保留执行权限，使用 `bash start.sh` 代替 `./start.sh`。

首次运行没有地图时使用建图模式：

```bash
./start.sh --mapping
```

完成建图并停止进程后，使用冻结地图定位：

```bash
./start.sh --localize
```

默认会打开 Gazebo、RViz2 和相机窗口。常用选项：

```bash
./start.sh --localize --no-camera       # Gazebo + RViz2
./start.sh --headless                   # 不打开 Gazebo/RViz2
./start.sh --localize --gcs 14550       # 转发 MAVLink 到 MP/QGC
./start.sh --localize --manual           # 等待 /mission/start
./start.sh --software-render             # 软件渲染
```

### Docker 无头运行

安装 Docker 后，可以在 Linux amd64 上使用统一的 Ubuntu 22.04 环境：

```bash
docker build -t aeromind-sim .
docker run --rm --init --shm-size=1g aeromind-sim
```

镜像从空白 Ubuntu 调用同一安装脚本，不挂载宿主机的 ROS、Python、SITL 或地图；默认无头运行并使用软件渲染。首次构建需要联网，并为 ROS、Gazebo 和 SITL 留出足够磁盘空间。容器中的构建和运行数据位于 `/var/tmp/aeromind-sim`；使用 `--rm` 退出后数据会删除。桌面可视化使用上面的原生安装流程。

Ubuntu 软件源下载慢时，可以在构建命令中追加 `--build-arg UBUNTU_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu`；ROS 软件源下载慢时追加 `--build-arg ROS_APT_URL=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu`。原生安装也支持先设置同名的 `ROS_APT_URL` 环境变量。软件镜像源使用相同的上游签名密钥，Gazebo 和 GitHub 仍需联网访问。

验证容器内建图、保存地图、重启和加载定位的完整流程：

```bash
docker run --rm --init --shm-size=1g --entrypoint bash aeromind-sim -c 'make smoke-mapping SMOKE_TIMEOUT=900'
```

手动模式需要在另一个终端执行：

```bash
source tools/activate.sh
ros2 topic pub /mission/start std_msgs/msg/Bool '{data: true}' --once
```

每次运行结束后按 `Ctrl+C`，或使用项目启动进程的进程组发送 `SIGTERM`。启动脚本使用锁文件保护固定端口；确认进程退出后再开始下一轮。

## 工程结构

| 路径 | 内容 |
| --- | --- |
| `src/ap_mission` | 任务状态机、路线约束和制导 |
| `src/ap_mavlink_bridge` | ROS 与 MAVLink、ArduPilot 参数和模式控制 |
| `src/ap_navigation` | 点云建图、地图定位、航点和地图工具 |
| `src/ap_perception` | ArUco、方框和气球识别 |
| `src/ap_simulation` | Gazebo 世界、机体模型、模拟传感器和 launch 文件 |
| `src/ap_gazebo_bridge` | Gazebo Transport 场景服务 |
| `tests/unit` | 不依赖 ROS 的算法测试 |
| `tests/ros` | ROS 消息和节点回归测试 |
| `tools` | 安装、构建、回归、发布和环境脚本 |
| `docs` | 架构、运行、点云和室内外切换说明 |
| `start.sh` / `Makefile` | 统一启动、构建和验证入口 |

## 外部私有目录

默认使用 `${XDG_CACHE_HOME:-$HOME/.cache}/aeromind-sim/`。该目录保存本机生成的数据，不随源码发布：

- `build/`、`install/`、`logs/`：ROS 构建和运行缓存；
- `dependencies/`：ArduPilot 等外部依赖；
- `maps/`：`arena.npz` 和 `.pcd` 地图；
- `artifacts/`：回归日志和验证结果；
- `releases/`：发布源码包。

需要改变位置时，在构建和运行前设置同一个绝对路径：

```bash
export COMPETITION_LOCAL_DIR=/srv/aeromind-sim-private
make build
./start.sh --check
```

该路径不能位于源码仓库内。源码移动后，重新设置路径并执行 `make build`。不要把私有目录、地图、日志、`build/`、`install/`、SITL 二进制或本机配置提交到 Git。

Makefile 还支持 `SITL_WORKDIR`、`ARDUCOPTER_BIN`、`ROS_DISTRO`、`PYTHON`、`SMOKE_TIMEOUT`、`SMOKE_VISUAL`、`SMOKE_ARGS` 和 `RUN_ARGS`。默认值和使用示例见[运行手册](docs/运行手册.md)；其中 `RUN_ARGS` 只用于向 `start.sh` 传递启动参数，`SMOKE_ARGS` 只用于回归脚本。

## 点云、航线和地面站

模拟雷达按 Livox Mid-360 的视场、频率、量程和噪声量级建模，输出标准 `sensor_msgs/PointCloud2` 话题 `/navigation/lidar/points`。实机接入时需要把 `livox_ros_driver2` 的 `CustomMsg` 转换为该话题，并填写实测的传感器外参。

RViz2 用于查看地图、注册点云、轨迹、ExternalNav 和场地标记。室内航点使用 `uav_map` 米制坐标，通过 YAML 文件编辑；MP/QGC 通过 `--gcs` 查看飞控状态、参数和日志，适合编辑已有 WGS84/GPS 参考的室外任务。当前没有自动的室内/室外定位源切换器。

详细步骤见：

- [运行手册](docs/运行手册.md)
- [开发环境与维护流程](docs/development.md)
- [架构与接口](docs/architecture.md)
- [室内点云建图与航点飞行](docs/indoor_navigation.md)
- [可视化、Mid-360 与室内外切换](docs/可视化与室内外切换.md)

## 验证

最近一次本机检查及建图、定位任务的结果见[验证记录](docs/validation.md)。

```bash
make check          # 代码、场景、独立测试和公开内容检查
make build          # 构建 ROS 工作空间
make test-all       # 独立测试和 ROS 回归测试
make smoke          # 单轮完整任务
SMOKE_VISUAL=1 make smoke-mapping  # 打开 Gazebo、RViz 和相机，验证建图、重启、定位两轮流程
```

`./start.sh --check` 还会实际导入已安装节点，检查 OpenCV ArUco API、NumPy/cv_bridge 图像转换、Gazebo 版本及模型资源。源码目录移动或换用另一个副本后必须重新 `make build`，启动脚本会拒绝使用其他位置的旧构建。

GitHub Actions 的 `Fresh Ubuntu / Install and run downloaded source` 作业从空白 Ubuntu 镜像安装，随后运行检查、ROS 回归及建图/定位两轮完整任务。该作业通过后，才能确认当前提交通过了干净环境的安装和运行验证；单独通过算法测试不能替代它。

完整仿真不能与同一台机器上的其他 Gazebo、SITL 或固定端口实例并行运行。回归日志写入私有目录的 `artifacts/`。验证必须看到任务状态机进入 `COMPLETE`；若出现 `ABORT`、`FAILSAFE`、`Traceback` 或节点崩溃，应先查看日志再判断结果。

## 发布和隐私检查

发布前只暂存 Git 仓库中的源码和公开资源：

```bash
git add -A
git diff --cached --stat
python3 tools/check_open_source_hygiene.py --staged
make check
make release
```

`make release` 会在私有目录的 `releases/aeromind-sim-source.zip` 生成不含 Git 元数据的源码包。不要直接压缩整个工作目录。发布前还应检查 staged diff 中没有用户名、绝对路径、内网地址、令牌、SSH 密钥、地图、日志或本机生成文件。

## 许可证

本项目自有代码采用 [MIT License](LICENSE)。ArduPilot、Gazebo 模型和其他第三方组件的许可与来源见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。漏洞报告方式见 [SECURITY.md](SECURITY.md)。
