# 参与贡献

欢迎通过 Issue 报告问题、通过 Pull Request 提交改进。较大的任务行为或接口变更请先说明用途与验证方式。

## 开发检查

环境、变量、测试和发布流程见[开发环境与维护流程](docs/development.md)。

独立开发可以使用虚拟环境：

```bash
source tools/workspace_env.sh
python3 -m venv "$COMPETITION_LOCAL_DIR/venv"
source "$COMPETITION_LOCAL_DIR/venv/bin/activate"
python3 -m pip install -r requirements-dev.txt
make check
```

修改 ROS 节点时，退出上述隔离环境，使用系统 Python 执行 `make build` 和 `make test-all`。修改路线、坐标、时间戳、控制或物理模型时，还应运行默认及光流模式的 `make smoke`，在 PR 中记录结果。

## 约定

- 纯算法测试放在 `tests/unit/`，依赖 ROS 的测试放在 `tests/ros/`。
- 点云模式航点使用 `src/ap_navigation/config/competition_waypoints.yaml`；真值/光流基线使用 `src/ap_mission/ap_mission/competition_route.py`。
- 比赛参数覆盖放在 `src/ap_simulation/config/sim_params.yaml`。
- 修改比赛模型模板后运行 `make model`，一起提交模板和生成模型。
- ArduPilot 修改保存为 `tools/patches/` 中的补丁，保留上游来源与许可证。
- 本地配置使用私有目录中的 `*.local.yaml`；构建、日志、地图、截图和凭据均放在仓库外，不提交。

暂存后运行 `python3 tools/check_open_source_hygiene.py --staged`，并检查 `git diff --cached`。模块与坐标说明见[架构说明](docs/architecture.md)。
