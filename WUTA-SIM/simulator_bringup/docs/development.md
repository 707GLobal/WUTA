# Simulator Bringup 开发记录

## RViz 真值地图配置

默认 RViz 配置为 `rviz/wuta_simulator.rviz`，安装后路径为：

```text
install/simulator_bringup/share/simulator_bringup/rviz/wuta_simulator.rviz
```

`MarkerArray` 显示必须使用 RViz 的 `Topic` 配置字段，并指向真实 ROS 话题。
若该字段缺失或写成无法被当前 RViz 读取的字段，RViz 会保存默认占位话题
`visualization_marker_array`。该话题不是本项目任何节点发布的内容；出现它表示
该显示没有订阅到预期话题。

| RViz 显示名称 | 正确话题 | 发布者 |
| --- | --- | --- |
| `Visible Cones` | `/sim/lidar/visible_cones` | `lidar_simulator` |
| `Loaded Track Cones (YAML Ground Truth)` | `/sim/lidar/track_cones` | `lidar_simulator` |
| `Perception Cones` | `/perception/lidar/cones_viz` | `lidar_detection_node` |
| `Cone Map` | `/mapping/cone_map_viz` | `cone_map_builder` |
| `Centerline` | `/planning/centerline_viz` | `boundary_detector_node` |
| `Control Target` | `/control/target_viz` | `controller_node` |
| `Final Waypoints` | `/planning/final_waypoints_viz` | `path_generator_node` |
| `Driven Trajectory (Smoothed Localization)` | `/planning/driven_trajectory_viz` | `path_generator_node` |
| `System Status (Mission and Ground Truth)` | `/system/status_viz` | `simulation_bridge` |

`Loaded Track Cones (YAML Ground Truth)` 使用 `Reliable + Transient Local`，并显式
启用 `track_cones` 与 `track_cone_info` 命名空间。RViz 的 Fixed Frame 必须为 `map`。

注意：FSD Pipeline 下的可视化仅在相应算法节点启动且产生数据后才会显示；
它们为空不代表 simulator 真值地图有问题。

`Driven Trajectory` 使用 `/localization/pose`，只在 marker 生成时做平滑/降采样；它反映定位
估计，不等同于 `/sim/ground_truth`。`Final Waypoints` 显示 path_generator 的当前目标路径。
状态 marker 显示 mission/state/complete、真值速度和位置。

## MissionState 所有权与手动就绪

`mission_manager_node` 是默认 FSD bringup 中唯一的 `/system/mission_state` 发布者。
`simulation_bridge` 仅发布 `/system/lidar_ready`、真值回退定位、`/system/start_command` 和状态
可视化；它订阅 MissionState，不再发布它。

启用 RViz 手动就绪：

```bash
./start_simulator.sh manual_ready:=true auto_start:=false --rviz
```

在 RViz 工具栏选择 **Publish Point** 后点击地图，bridge 收到 `/clicked_point`，开始发布 ready，
状态机进入 `READY`。再发布 `/system/start_command=true` 才进入 `EXPLORE`。

## README 自动启动脚本

仓库根目录的 `start_simulator.sh` 是 README 推荐入口。它按如下顺序处理 overlay：

```text
WUTA-FSD/ros2_ws/install/setup.bash
-> WUTA-SIM/install/setup.bash
-> ros2 launch simulator_bringup simulator.launch.py
```

默认不会启动 RViz。必须传入 `--rviz`，脚本才会追加 `launch_rviz:=true` 并加载上述
默认配置：

```bash
cd /home/starry1n/WUTA
./start_simulator.sh --rviz
```

`--skip-build` 只使用已有 install 空间；修改 Python 节点、launch 或 `.rviz` 后，
若未重新构建，仍会加载旧版安装文件。首次验证修改时使用：

```bash
./start_simulator.sh --clean --rviz
```

之后可使用：

```bash
./start_simulator.sh --skip-build --rviz
```

应先完全关闭旧的 ROS/RViz 进程，再启动新实例，避免手工打开的旧 RViz 配置或不同
ROS 环境导致话题看似缺失。

## 默认 INS 与融合定位链

`WUTA-SIM/wuta-ins-simulator` 是主仓库记录的 Git submodule。首次拉取或更新主仓库后，
先在根目录执行：

```bash
git submodule update --init --recursive
```

`simulator.launch.py` 默认启动 INS 与融合定位。INS 将 `/sim/ground_truth` 加噪后以 20 Hz
发布 `/cg410/odometry`；KISS-ICP 处理 `/hesai/pandar` 并发布 `/kiss/odometry`；EKF 融合两者
并发布 `/odometry/filtered`，localization_manager 最终发布 `/localization/pose`。

TF 所有权固定为：bringup 发布静态同原点 `map -> odom` 与 `base_link -> lidar`，EKF 发布唯一
动态 `odom -> base_link`，KISS 不发布 TF。需要在不接入 INS、KISS-ICP 与 EKF 的情况下进行
真值回退时，只需设置 `use_ground_truth_localization:=true`；启动文件会自动关闭 INS 与融合定位：

```bash
ros2 launch simulator_bringup simulator.launch.py \
  use_ground_truth_localization:=true
```

不要单独设置 `launch_ins:=false`：此时 EKF 仍会启动却没有 INS 输入。也不要单独设置
`launch_localization:=false` 后启动 FSD：控制器将得不到 `/localization/pose`。若只需查看
传感器和 RViz，可使用：

```bash
./start_simulator.sh --skip-build --rviz \
  launch_fsd:=false use_ground_truth_localization:=true
```

## 配置检查

构建 `simulator_bringup` 后，可确认安装配置不含 RViz 默认占位话题：

```bash
cd /home/starry1n/WUTA/WUTA-SIM
colcon build --packages-select simulator_bringup --symlink-install
rg 'visualization_marker_array' \
  install/simulator_bringup/share/simulator_bringup/rviz/wuta_simulator.rviz
```

最后一条命令不应有输出。
