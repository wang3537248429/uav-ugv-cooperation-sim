# 第三方组件声明

## PX4 Gazebo x500 模型

`src/ap_simulation/models/x500/` 和 `x500_base/` 来自 [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)，固定于提交 `e00d3b9cde682dbcb3bf6f30a2f2b8ef4325dae8`。目录保留上游 BSD 3-Clause 许可证；比赛模型由本项目生成工具派生。

## ArduPilot

安装脚本将官方 [ArduPilot](https://github.com/ArduPilot/ardupilot) 下载到仓库外私有目录的 `dependencies/ardupilot/`，固定于提交 `b72075518ff1409438a817a1dd9eea4828d28642`，遵循上游 GPL-3.0-or-later 许可证。`tools/patches/` 中的补丁用于仿真初始化和 `OPTICAL_FLOW_RAD` 接口；应用补丁后的 ArduPilot 仍遵循其上游许可。

本仓库的 MIT 许可证不替代上述组件各自的许可证。发布时请保留第三方许可证和来源信息。
