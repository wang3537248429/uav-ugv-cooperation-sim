# 验证记录

验证日期：2026-11-22。环境为 Ubuntu 22.04 amd64、Python 3.10、ROS 2 Humble、Gazebo Sim 7，使用本机已经安装的依赖。

| 检查 | 结果 |
| --- | --- |
| `make check` | 63 项独立测试、代码风格、模型一致性、场景尺寸及公开内容检查通过 |
| `make test-ros` | 23 项 ROS 回归测试通过 |
| `bash start.sh --check` | 节点导入、ArUco、cv_bridge、Gazebo 与场景资源检查通过 |
| `LIBGL_ALWAYS_SOFTWARE=1 make smoke-mapping SMOKE_TIMEOUT=600` | 建图与保存地图定位两轮任务均进入 `COMPLETE`，地图保存、加载和只读检查通过 |

建图轮得到 16,687 个地图点，定位轮使用同一份地图。独立监视器相对仿真真值记录的位姿位置 RMS 误差分别为 0.0504 m 和 0.0906 m；这是一组固定场地仿真的运行结果，不代表其他环境的精度或实机性能。

本轮使用已有系统依赖，未重新完成空白 Ubuntu 的联网安装。仓库已配置 GitHub Actions 的干净安装作业；是否通过应以对应提交的运行结果为准。完整日志保留在本机私有工作区，不随源码发布。
