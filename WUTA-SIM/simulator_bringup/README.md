# simulator_bringup

`simulator_bringup` 是 WUTA 仿真系统的统一 ROS 2 启动包。各模拟器仍是独立包；
本包通过包含它们各自的 launch 文件进行编排，并可选启动 WUTA-FSD Level A
闭环。默认定位链由 `ins_simulator`、KISS-ICP、EKF 和 localization_manager 组成：
INS 将 ground truth 加噪后发布 `/cg410/odometry`，KISS-ICP 从 `/hesai/pandar` 生成
`/kiss/odometry`，EKF 融合后经 localization_manager 发布 `/localization/pose`。

## Dependency order

1. `vehicle_model` 先启动，接收 WUTA-FSD 的
   `autoware_msgs/msg/Command`，发布 `/sim/ground_truth`。
2. `can_simulator` 和 `lidar_sim` 在 ground truth 源启动后再启动。
3. `can_simulator`、`lidar_sim` 与 `ins_simulator` 默认启动；INS 发布模拟 CG-410
   里程计。
4. 随后默认启动 KISS-ICP、EKF 和 localization_manager，产生统一定位输出。
5. 启用 `launch_fsd` 时，WUTA-FSD 按数据流顺序启动：
   `lidar_detection` -> `cone_map_builder` -> `boundary_detector` ->
   `mission_manager` -> `path_generator` -> `controller`.

`simulation_bridge` 默认提供就绪状态以及（`auto_start:=true` 时）
`/system/start_command=true` 仿真出发输入。`mission_manager` 是唯一的
`/system/mission_state` 发布者：就绪后进入 `READY`，收到开始输入进入 `EXPLORE`，再由
控制器的 `/system/mission_complete`（`std_msgs/msg/Bool=true`）进入 `FINISH`。
默认的 `/localization/pose` 与动态 `odom -> base_link` TF 由融合定位链发布；bridge 的真值
pose/TF 仅在 `use_ground_truth_localization:=true` 时启用。

RViz 默认显示 `/system/status_viz`：任务模式、`EXPLORE/FINISH` 状态、
`/system/mission_complete` 对应的完成状态以及 `/sim/ground_truth` 的速度和位置。该话题只用于调试显示，
不参与控制或定位。

### RViz 手动就绪调试

需要手动控制状态机就绪时，以 `manual_ready:=true` 启动。bridge 会在 RViz 点击前保持
`/system/lidar_ready=false` 与 `/system/localization_ready=false`；选择 RViz 工具栏的
**Publish Point** 后在地图任意位置点击一次，即锁存就绪。配合 `auto_start:=false` 可先停在
`READY`，再从终端发布 `/system/start_command=true`；若保留 `auto_start:=true`，点击后会自动进入
`EXPLORE`。

## Build

推荐从仓库根目录使用一键脚本。它会先调用 WUTA-FSD 自带的
`ros2_ws/build_ws.sh` 完整构建 16 个 FSD 包，再构建模拟器 overlay：

```bash
cd /path/to/WUTA
./start_simulator.sh
```

### 一键脚本用法

| 参数 | 作用 |
|---|---|
| 无参数 | 增量构建完整 WUTA-FSD 和模拟器，然后启动完整闭环 |
| `--clean` | 清理两个工作区后重新完整构建并启动 |
| `--build-only` | 完成构建后退出，不启动 ROS 节点 |
| `--skip-build` | 使用已有安装空间直接启动 |
| `--rviz` | 启动时同时打开 RViz2 默认可视化配置 |
| `--config PATH` | 读取 YAML 构建/启动默认参数；命令行标志与 `name:=value` 可覆盖其中任意项 |
| `-h` / `--help` | 显示脚本帮助 |
| `--` | 后续参数全部原样传给 ROS launch |

构建和启动示例：

```bash
# 默认：增量构建并启动模拟器和 WUTA-FSD
./start_simulator.sh

# 默认闭环，并同时打开 RViz2
./start_simulator.sh --rviz

# 清理两个工作区，完整重建后启动
./start_simulator.sh --clean

# 只构建，不启动
./start_simulator.sh --build-only

# 清理后只构建，用于验证完整构建
./start_simulator.sh --clean --build-only

# 使用已有构建结果启动完整闭环
./start_simulator.sh --skip-build

# 使用已有构建结果启动完整闭环，并打开 RViz2
./start_simulator.sh --skip-build --rviz

# 只启动模拟器，不启动 WUTA-FSD 算法链
./start_simulator.sh --skip-build launch_fsd:=false

# 选择赛道和任务模式
./start_simulator.sh track_file:=skidpad mission_mode:=skidpad

# Skidpad 完整闭环并打开 RViz（起点自动为 -15 m）
./start_simulator.sh --rviz track_file:=skidpad mission_mode:=skidpad

# Acceleration：-0.30 m 起步，75 m 计时，100 m 停止区
./start_simulator.sh --rviz track_file:=acceleration mission_mode:=acceleration

# 真值定位调试：不启动 INS、KISS-ICP 或 EKF
./start_simulator.sh --skip-build use_ground_truth_localization:=true

# 使用另一套启动默认值；命令行参数优先
./start_simulator.sh --config config/simulator_defaults.yaml \
  track_file:=skidpad mission_mode:=skidpad

# 调整依赖阶段之间的启动间隔
./start_simulator.sh startup_delay:=1.0

# 自定义车辆初始位姿
./start_simulator.sh -- \
  track_file:=/path/to/track.yaml \
  start_x:=1.0 start_y:=2.0 start_yaw:=0.5
```

手动构建时，必须先完整构建并加载 WUTA-FSD，再构建模拟器 overlay。这样
`vehicle_model` 才能找到 `autoware_msgs`：

```bash
cd /path/to/WUTA/WUTA-FSD/ros2_ws
./build_ws.sh
source install/setup.bash

cd ../../WUTA-SIM
colcon build --base-paths . --symlink-install \
  --packages-up-to simulator_bringup
source install/setup.bash
```

## Run

```bash
ros2 launch simulator_bringup simulator.launch.py
```

Useful overrides:

```bash
ros2 launch simulator_bringup simulator.launch.py launch_fsd:=false
ros2 launch simulator_bringup simulator.launch.py launch_rviz:=true
ros2 launch simulator_bringup simulator.launch.py track_file:=skidpad mission_mode:=skidpad
ros2 launch simulator_bringup simulator.launch.py startup_delay:=1.0
ros2 launch simulator_bringup simulator.launch.py use_ground_truth_localization:=true
ros2 launch simulator_bringup simulator.launch.py auto_start:=false
ros2 launch simulator_bringup simulator.launch.py \
  track_file:=/path/to/track.yaml start_x:=1.0 start_y:=2.0 start_yaw:=0.5
```

`track_file` 和 `mission_mode` 应选择同一比赛项目。若赛道起点不是原点，还需传入
一致的 `start_x`、`start_y` 和 `start_yaw`。

定位相关参数如下。`use_ground_truth_localization:=true` 是唯一推荐的“无需 INS/EKF”调试
方式：启动文件会自动关闭 INS、KISS-ICP、EKF 与 localization_manager，并由 bridge 发布
真值 `/localization/pose` 和 `map -> base_link`。不要只设置 `launch_ins:=false`，否则 EKF
失去 INS 输入；也不要只设置 `launch_localization:=false` 后启动 FSD，因为控制链将没有
`/localization/pose`。

| 场景 | 参数 | 结果 |
| --- | --- | --- |
| 默认闭环 | 不传定位参数 | INS + KISS-ICP + EKF + localization_manager，EKF 发布 `odom -> base_link` |
| 真值定位调试（不接 INS/EKF） | `use_ground_truth_localization:=true` | bridge 发布真值 pose/TF；INS 与融合定位自动关闭 |
| 仅仿真传感器/RViz | `launch_fsd:=false use_ground_truth_localization:=true` | 不启动 FSD 感知、规划、控制；保留真值传感器与 TF |

其他常用 launch 参数：`auto_start:=false` 停留在 `IDLE` 等待外部任务状态；`start_x:=auto`
会在 Skidpad 自动选用 `-15 m`、Acceleration 自动选用 YAML 规定的 `-0.30 m`、Trackdrive
选用 `0 m`；可用 `wheel_base`、`max_steer_angle`
和 `vehicle_dt` 覆盖车辆模型参数。

### 启动默认参数配置

根目录的 [`config/simulator_defaults.yaml`](../../config/simulator_defaults.yaml) 是
`start_simulator.sh` 的默认参数来源。`build` 段包含 `clean`、`skip_build`、`build_only`；
`launch_arguments` 段包含当前 `simulator.launch.py` 声明的全部参数：赛道/任务、FSD、定位、
RViz、车辆模型和初始位姿。脚本不依赖 `yq` 或 PyYAML，而是使用内置的扁平 YAML 解析器。
命令行参数始终优先，例如 `track_file:=skidpad` 仅覆盖配置中的 `track_file`。使用另一份配置可传入：

```bash
./start_simulator.sh --config /path/to/simulator_defaults.yaml --rviz
```

## RViz2 visualization

推荐直接用一键脚本启动完整闭环和 RViz2：

```bash
cd /path/to/WUTA
./start_simulator.sh --rviz
```

若已经构建完成，可跳过构建：

```bash
cd /path/to/WUTA
./start_simulator.sh --skip-build --rviz
```

该命令等价于启动 `simulator_bringup` 时传入 `launch_rviz:=true`，并加载默认
RViz 配置：

```bash
ros2 launch simulator_bringup simulator.launch.py launch_rviz:=true
```

默认配置文件安装在：

```text
share/simulator_bringup/rviz/wuta_simulator.rviz
```

源码路径为：

```text
WUTA-SIM/simulator_bringup/rviz/wuta_simulator.rviz
```

默认 RViz 设置：

| Display | Topic | 用途 |
|---|---|---|
| `TF` | `map -> odom -> base_link -> lidar` | 坐标系关系 |
| `Odometry` | `/sim/ground_truth` | 车辆真值位置 |
| `PointCloud2` | `/hesai/pandar` | LiDAR 仿真点云 |
| `MarkerArray` | `/sim/lidar/visible_cones` | LiDAR 当前可见锥筒 |
| `MarkerArray` | `/sim/lidar/track_cones` | 从赛道 YAML 读取的全量锥筒地图 |
| `MarkerArray` | `/perception/lidar/cones_viz` | 感知检测锥筒 |
| `MarkerArray` | `/mapping/cone_map_viz` | 建图后的全局锥筒地图 |
| `MarkerArray` | `/planning/centerline_viz` | 规划中心线 |
| `MarkerArray` | `/control/target_viz` | 控制目标/预瞄点 |

RViz 的 `Fixed Frame` 已配置为 `map`。`/hesai/pandar` 点云已配置为
`Best Effort` QoS，以匹配传感器数据发布方式。

只可视化模拟器、不启动 WUTA-FSD 算法链时：

```bash
./start_simulator.sh --skip-build --rviz launch_fsd:=false
```

此时可见的主要 topic 是 `/sim/ground_truth`、`/hesai/pandar` 和
`/sim/lidar/visible_cones`、`/sim/lidar/track_cones`；感知、建图、规划和控制相关
可视化 topic 不会发布。

也可以手动启动 RViz2：

```bash
source /opt/ros/humble/setup.bash
source /path/to/WUTA/WUTA-FSD/ros2_ws/install/setup.bash
source /path/to/WUTA/WUTA-SIM/install/setup.bash
rviz2 -d /path/to/WUTA/WUTA-SIM/install/simulator_bringup/share/simulator_bringup/rviz/wuta_simulator.rviz
```

常用检查命令：

```bash
ros2 topic list
ros2 topic hz /hesai/pandar
ros2 topic hz /perception/lidar/cones
ros2 topic hz /mapping/cone_map
ros2 topic hz /planning/centerline
ros2 run tf2_tools view_frames
```

如果 RViz 提示 `No transform from [lidar] to [map]`，先确认仿真仍在运行，并检查静态
`map -> odom`、EKF 的 `odom -> base_link`、以及静态 `base_link -> lidar`。如果只缺点云显示，检查 `/hesai/pandar` Display 的
`Reliability Policy` 是否为 `Best Effort`。
