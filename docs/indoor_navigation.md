# 室内点云建图与航点飞行

当前实现面向小型固定场地仿真：Gazebo 原生三维雷达 → `PointCloud2` → 点到平面 ICP → 地图位姿 → ExternalNav → ArduPilot 航点控制。没有 GPS 定位，也不向匹配器提供 Gazebo 位姿、场景几何或航点作为定位提示。

## 建图、保存与加载

先 `make build`。所有地图和日志均放在仓库外的私有目录，`source tools/activate.sh` 后可通过 `$COMPETITION_LOCAL_DIR` 引用。首次建图沿现有保守比赛航线飞行，地图会随观测扩充；这不是未知环境自动探索。

```bash
./start.sh --mapping
# 无界面：./start.sh --mapping --headless
```

默认约 24 秒后自动开始仿真任务。RViz 的 `Saved or Building 3D Map` 显示体素点云，`Registered LiDAR Scan` 显示当前匹配点云，`Point Cloud Localization Path` 显示估计轨迹。它们与原有场地 Marker 有独立的开关。

地图每 15 秒自动保存，正常退出也会保存；至少四帧成功匹配后才允许保存。完成采集后按 `Ctrl+C` 停止。默认文件位于 `${XDG_CACHE_HOME:-$HOME/.cache}/aeromind-sim/`；以下变量需先执行 `source tools/activate.sh`：

- `$COMPETITION_LOCAL_DIR/maps/arena.npz`：带版本、坐标系与分辨率的定位地图。
- `$COMPETITION_LOCAL_DIR/maps/arena.pcd`：通用 XYZ 点云导出。

下次启动使用冻结地图：

```bash
./start.sh --localize
./start.sh --localize --manual   # 查看地图和定位，等待人工启动任务
```

定位模式不会更新地图或保存地图；文件缺失/损坏会在启动 Gazebo 前报错。建图模式拒绝覆盖已有地图，另建地图请显式选择路径：

```bash
source tools/activate.sh
./start.sh --mapping --map "$COMPETITION_LOCAL_DIR/maps/arena-v2.npz"
./start.sh --localize --map "$COMPETITION_LOCAL_DIR/maps/arena-v2.npz"
```

## 在地图中预设航线

复制航点文件后编辑，保留比赛任务标签及顺序，可以在 `TAKEOFF` 到 `BALLOON_ALT` 之间增加唯一命名的过渡点；识码和等待无人车结束后会继续这些点。扎球后的返航采用固定安全走廊，`BALLOON_ALT` 与 `LAND` 之间不接受过渡点：

```bash
source tools/activate.sh
cp src/ap_navigation/config/competition_waypoints.yaml "$COMPETITION_LOCAL_DIR/route.local.yaml"
./start.sh --localize --manual waypoint_file:="$COMPETITION_LOCAL_DIR/route.local.yaml"
```

`position: [x, y, z]` 的单位是米，地图 `uav_map` 中 X 向前、Y 向左、Z 向上。原点是首次建图时选定的起飞机体参考点，不是 GPS 经纬度，也不是地板高度。程序在飞控边界转换为 NED/FRD；不要手动再反转 Y/Z。

航点在任务节点启动时读取，编辑后需要重新启动。高度直接取文件值。查看地图时可用 RViz 的 Measure 工具确认距离。航点文件只改变 UAV 路线，场地和无人车轨迹不会跟着移动。

可检查航线各段与已观测地图点的距离：

```bash
source tools/activate.sh
ros2 run ap_navigation map_tool "$COMPETITION_LOCAL_DIR/maps/arena.npz" \
  --route "$COMPETITION_LOCAL_DIR/route.local.yaml" --clearance 0.18
ros2 service call /navigation/save_map std_srvs/srv/Trigger '{}'
```

距离检查覆盖线段，不只检查端点；它检查航线中心与已有观测点的距离，不保证未知区域可通行，也不能替代按实际机身、针具和误差预算设置安全余量。保存服务只在建图模式有效。手动开始命令见仓库 README。

飞行中可请求受控降落：`ros2 topic pub /mission/land std_msgs/msg/Bool '{data: true}' --once`。这会将任务标记为未完成；点云模式等待飞控接地反馈后上锁，并确认姿态稳定。

## 定位参数和失败处理

`src/ap_navigation/config/navigation.yaml` 设置雷达外参、范围、体素大小、匹配质量和超时。需要定制时复制到私有目录的 `navigation.local.yaml`，启动追加 `navigation_params:="$COMPETITION_LOCAL_DIR/navigation.local.yaml"`。

`initial_pose` 是地图中的近似起点 `[x,y,z,roll,pitch,yaw]`，角度为弧度。当前依赖固定起飞区及近似已知朝向，ICP 只做邻域内配准，不支持从任意未知地点自动寻找全局位置。返航/降落仍以地图原点为起降点，修改起飞位置前必须一并检查任务定义。

比赛场地的围墙顶端高度一致，默认配置用 `upper_structure_quantile: 0.995` 从点云上边界计算受限的垂向初值，解决 Mid-360 下视角较窄、起飞后地面回波不足造成的高度漂移。它只为 ICP 提供初值，仍须通过重叠度、残差和跳变检查。换到墙高不一、上方结构变化或室外场景时，将该参数设为 `0.0`；通用节点默认关闭这一场地假设。实机还需要 IMU 融合与运动畸变补偿，当前实现不承担这些功能。

启动默认将每个 OpenCV/BLAS 进程限制为一个工作线程，避免多进程线程过量争用 CPU。若修改 `worker_threads`，应同时观察 Gazebo 实时速率和完整回归结果。

点云模式的飞行速度与加减速配置在 `src/ap_simulation/config/pointcloud_flight.yaml`，当前水平指令上限为 0.16m/s、垂向指令上限为 0.04m/s。可复制文件后通过 `flight_profile:="$COMPETITION_LOCAL_DIR/flight.local.yaml"` 指定配置；提高速度后需要重新跑完整任务和定位误差检查。该配置不会提高小车速度。

`--truth` 对照任务复用这套航线、飞行参数、GUIDED 起飞和受限下降流程，定位输入改为 Gazebo 真值。它不经过旧光流基线的低空 RC 交权过程，便于比较定位输入对飞行结果的影响。

点云模式使用独立的 200Hz 机体物理位姿流生成 SITL 传感器状态，按传感器时间推进飞控并计算速度；这些物理真值不进入点云匹配器。EKF 原点及 HOME 仅定义飞控坐标参考，不是 GPS 测量，确认后使用飞控原生 GUIDED 起飞；启动阶段若 EKF 重置丢失早期原点，会重新初始化并等待确认。

至少三帧连续有效匹配后才发布可用定位；退化场景、重叠不足、残差超限、异常位姿跳变和点云超时会撤销 `/ap/external_nav/ready`，拒绝坏位姿，交由现有任务超时/降落逻辑处理。原地扫描一块平墙不能提供完整六自由度约束。

## 验证与边界

```bash
make check
make build
make test-all
SMOKE_VISUAL=1 make smoke-mapping
```

最后一个命令执行两次独立 Gazebo/SITL 任务：先建图，再重新启动并加载同一地图定位飞行，最后比较地图哈希确认未改写。独立监视器用真值检查飞行范围及定位误差，点云定位误差超过 0.40m 会失败；真值不会反馈给点云定位器。结果在私有目录的 `artifacts/navigation-smoke-*/`。

该实现是轻量点云建图/定位基线，没有全局回环优化、自动路径规划或动态障碍避让。当前比赛场景、相机、无人车和气球确认仍保留原有仿真简化；新增地图链路不代表扎球实物闭环或官方比赛验收完成。

三维雷达目前是虚拟传感器。现有普通摄像头、光流或测距模块不能直接当成等价三维点云输入。实物传感器选型、树莓派性能、飞控接口及正式规则允许范围仍须另行验证，当前启动入口只用于仿真。
