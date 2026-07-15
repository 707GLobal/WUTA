# WUTA 部署与运行手册

## 1. 部署边界

当前仓库可直接部署包含 INS、KISS-ICP 与 EKF 的软件仿真闭环。真实车辆部署需要额外提供
与配置一致的 LiDAR、CG-410/INS、CAN/VCU 驱动；
这些驱动不在当前源码中。不要把 simulator 的 `/sim/ground_truth` 或
`simulation_bridge` 当作实车定位源。

最低运行环境：Ubuntu + ROS 2 Humble、PCL、tf2、RViz2、Python `numpy`/`yaml`，以及
package.xml 声明的依赖。NDT/KISS 路径还需要 PCD 地图、Eigen 与对应定位组件。

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd /home/starry1n/WUTA
git submodule update --init --recursive
./start_simulator.sh --build-only
```

该脚本先调用 `WUTA-FSD/ros2_ws/build_ws.sh`，再使用
`colcon build --base-paths . --symlink-install --packages-up-to simulator_bringup` 构建
`WUTA-SIM`。手动构建时必须保持相同 source 顺序，详见 `DEVELOPMENT.md`。

## 3. Configuration

| 用途 | 实际文件 |
| --- | --- |
| 仿真 LiDAR | `WUTA-SIM/perception_simulation/config/lidar_simulator.yaml` |
| 仿真 INS（submodule） | `WUTA-SIM/wuta-ins-simulator/launch/ins_simulator.launch.py` |
| 赛道 | `WUTA-SIM/perception_simulation/tracks/{trackdrive,skidpad,acceleration}.yaml` |
| 感知/地图/规划/控制 | 各 FSD package 的 `config/*.yaml` |
| KISS-ICP | `WUTA-FSD/ros2_ws/src/localization/kiss_icp_wrapper/config/kiss_icp_hesai128.yaml` |
| EKF | `WUTA-FSD/ros2_ws/src/localization/localization_manager/config/ekf.yaml` |
| NDT/保存地图 | `WUTA-FSD/ros2_ws/src/localization/ndt_localization/config/ndt_localization.yaml` |
| RViz | `WUTA-SIM/simulator_bringup/rviz/wuta_simulator.rviz` |

部署前为真实硬件复核 frame、话题名、QoS、LiDAR 外参和 EKF 输入。NDT 默认地图路径为
`/tmp/wuta_lidar_map.pcd`；锥筒地图保存默认 `/tmp/wuta_cone_map.yaml`，应替换为有权限且
可追溯的持久化路径。

## 4. Launch

完整仿真并打开项目 RViz：

```bash
cd /home/starry1n/WUTA
./start_simulator.sh --rviz
```

仅使用已有构建：

```bash
./start_simulator.sh --skip-build --rviz
```

常见 launch 覆盖：

```bash
ros2 launch simulator_bringup simulator.launch.py \
  track_file:=skidpad mission_mode:=skidpad launch_rviz:=true
ros2 launch simulator_bringup simulator.launch.py launch_fsd:=false
ros2 launch simulator_bringup simulator.launch.py use_ground_truth_localization:=true
```

第二条不启动 FSD 感知-控制链，但默认仍会运行定位链，因此 FSD Pipeline 可视化为空是预期行为。
第三条是显式真值回退：设置 `use_ground_truth_localization:=true`，启动脚本会自动关闭
INS/融合定位，并由 bridge 发布 `/localization/pose` 和 TF。

## 5. Runtime Check

```bash
ros2 node list
ros2 topic list -t
ros2 topic info -v /sim/lidar/track_cones
ros2 topic echo --once /sim/lidar/track_cones
ros2 topic echo --once /cg410/odometry
ros2 topic echo --once /kiss/odometry
ros2 topic echo --once /odometry/filtered
ros2 topic echo --once /localization/pose
ros2 run tf2_tools view_frames
```

最小闭环应具备 `vehicle_model`、`lidar_simulator`、`can_simulator`、
`ins_simulator`、`kiss_icp_node`、`ekf_node`、`localization_manager`、`simulation_bridge` 与
`map -> odom -> base_link -> lidar`。完整默认链再包含
`lidar_detection_node`、`cone_map_builder`、`boundary_detector_node`、
`path_generator_node`、`controller_node`。

RViz Fixed Frame 设为 `map`。若 `Visible Cones` 报 transform 错误，检查
`/odometry/filtered`、静态 `map -> odom`、EKF 的 `odom -> base_link` 和 static
`base_link -> lidar`；若 MarkerArray topic 显示为
`visualization_marker_array`，重新加载本仓库安装的 RViz 配置。

## 6. 真实硬件接入检查

1. 将 LiDAR 驱动输出与 `/hesai/pandar` 类型/时间戳/frame 对齐，或通过参数重映射；
2. 提供 `base_link -> lidar` 静态外参与完整定位 TF；
3. 将真实 INS 输出映射到 `/cg410/odometry`，并复核其 frame/covariance 后再启用当前 EKF；

## 7. Skidpad 实车控制标定

当前 `controller` 可以作为实车 Pure Pursuit 的软件起点，但不能将仿真参数直接视为可上车标定：
`vehicle_model` 是无执行器延迟、无轮胎侧偏、速度瞬时跟随的运动学自行车模型。实车接入前至少应：

1. 实测并填写有效轴距、最大前轮转角、转向零位/正负方向和转向角—方向盘/CAN 指令映射；确认 `/control/command.angle` 的单位为度。
2. 以低速闭环测量转向执行延迟、转向速率限制、制动减速度和纵向速度跟踪误差；将速度命令替换或接入 VCU 的闭环速度/制动控制，而非假设瞬时跟随。
3. 标定 `base_link -> lidar` 外参、定位延迟与时间戳，并验证定位输出在控制周期内连续；横向控制对时延和航向误差非常敏感。
4. 从低速逐级提高 Skidpad 速度，同时记录横向误差、转角饱和率和轮胎工况；`skidpad_lookahead` 的 3.0 m 只是当前仿真的初值，应按实车响应调节。
5. 增加独立安全链：急停、制动/转角限幅、通信超时停车、人工接管和赛道边界保护。完成封闭场地验证前不得用于无人实车运行。
4. 审核 `map_saver` 的 TODO：其源码暂假定点云已在 map frame，不能未经修复直接用于
   多 frame 实车建图；
5. 验证紧急制动与 VCU/CAN：`mission_manager` 的 CAN 车检发送当前未实现，不能作为安全
   功能部署依据。
