# WUTA ROS 2 内部设计

## 1. 模块设计与代码位置

| 模块 | 核心类 / 位置 | 初始化与运行 |
| --- | --- | --- |
| 车辆仿真 | `VehicleModel`；`WUTA-SIM/vehicle_model/src/vehicle_model/vehicle_model/vehicle_model.py` | 订阅命令；按 `dt` 积分自行车模型并发布 Odometry |
| LiDAR 仿真 | `LidarSimulatorNode`、`LidarSimulator`；`WUTA-SIM/perception_simulation/lidar_sim/` | 加载 YAML；启动时发布静态地图；定时生成 scan |
| 仿真桥接 | `SimulationBridge`；`WUTA-SIM/simulator_bringup/simulator_bringup/simulation_bridge.py` | 发布就绪/任务状态；真值 pose/TF 仅作显式回退 |
| 感知 | `LidarDetectionNode` 与 `TraditionalDetector`；`WUTA-FSD/.../perception/lidar_detection/` | PointCloud2 转 PCL，检测后发布 ConeArray |
| 锥筒地图 | `ConeMapBuilder`；`WUTA-FSD/.../mapping/cone_map_builder/` | 用 TF 变换检测，去重、颜色分配、闭环和定时发布 |
| 边界/中心线 | `BoundaryDetectorNode` 与 `PathSearch`；`WUTA-FSD/.../planning/boundary_detector/` | 对局部锥筒执行 Delaunay/中点搜索 |
| 路径 | `PathGeneratorNode`；`WUTA-FSD/.../planning/path_generator/` | Trackdrive 转发中心线；Skidpad 固定四圈、退出和停车路径；Acceleration 解析直线路径 |
| 控制 | `ControllerNode`、`PurePursuit`、`TwistFilter`；`WUTA-FSD/.../control/controller/` | 定时计算目标点、转向与速度限幅 |
| 定位 | `INSSimulator`、`kiss_icp_node`、`ekf_node`、`LocalizationManager`、`NdtLocalization` | 默认融合 INS 与 KISS-ICP；NDT 仅在 NDT 模式运行 |

所有自定义 C++ 节点以 `rclcpp::spin(std::make_shared<...>())` 运行；Python 节点以
单节点 `rclpy.spin(node)` 运行。它们没有 ROS lifecycle 状态机，启动、运行和
shutdown 分别对应构造函数、回调/定时器和 `destroy_node()`/`rclcpp::shutdown()`。

## 2. Algorithm Design

### 2.1 LiDAR 与真值地图

```text
track YAML + vehicle pose
  -> 视场/距离/前向筛选
  -> 可选锥筒遮挡判断与检测概率
  -> 锥筒表面点 + 高斯噪声 + 地面点
  -> PointCloud2（lidar frame）
```

YAML 中的全部锥筒另作为静态 `map` marker 发布。marker 的圆柱高度按 YAML 底面位置
居中，颜色/尺寸来自锥筒规格，文本 marker 给出编号、类型和尺寸。此真值话题不进入
FSD 建图链路。

### 2.2 传统锥筒检测与地图

检测器以高度阈值或 RANSAC 去地面、体素下采样、欧氏聚类，再以宽度/高度/距离筛选。
`ConeMapBuilder` 以消息时间戳查询 `map <- sensor_frame`，在 `merge_distance` 内对坐标
做命中次数加权平均；未确认锥筒在发布前由 `min_hit_count` 过滤。颜色可按车辆航向的
左右叉积分配。达到最少确认数量且车辆离开又回到起点阈值内时，地图闭环并保存 YAML。

### 2.3 规划与控制

Trackdrive：`BoundaryDetectorNode` 提取 lookahead 范围内的蓝/黄/未知锥筒，构建 Delaunay
图并搜索中点序列，输出 `Lane`。Skidpad 按 `skidpad_start_*` 固定 map 参考生成：右环顺时针
两圈、左环逆时针两圈，再沿进入方向退出 25 m 并制动；不以实时定位位姿重建。Acceleration
按航向每米生成一个点并在最后 10 m 降速。控制器只在 `EXPLORE` 或 `RACE` 启用：通常以速度
比例的 lookahead 选择目标点，Pure Pursuit 计算命令，`TwistFilter` 再执行车辆约束/安全
过滤。Skidpad 专用 `skidpad_lookahead=3.0 m` 覆盖通用动态前视（5 m/s 下原本为 10 m）：
圆的半径仅 9.125 m，10 m 前视会跨越交叉点的曲率切换，从而在第一/三圈切入内侧、或在第四圈
出口过早卸载转向。自交叉 Skidpad 路径通过单调前向进度索引保持圈序；停车后控制器通知 bridge
发布 `FINISH`。

### 2.4 定位模式

`ins_simulator` 默认以 20 Hz 缓存 `/sim/ground_truth`，向位置、yaw、速度和角速度注入
可配置高斯噪声及协方差，再发布 `/cg410/odometry`。KISS-ICP 将 `/hesai/pandar` 注册为
`/kiss/odometry`；其 `lidar_odom_frame=odom` 且不发布 TF。EKF 融合这两路 Odometry，发布
`/odometry/filtered` 和唯一的动态 `odom -> base_link`。bringup 另发布静态同原点
`map -> odom` 与 `base_link -> lidar`，避免 TF 发布者冲突。`LocalizationManager` 在
`LOC_KISS_ICP` 时将 `/odometry/filtered` 转为 `/localization/pose`，在 `LOC_NDT` 时接受
`/ndt/pose`。`use_ground_truth_localization:=true` 是调试回退，不应与默认 EKF TF 并用。
NDT 读取 PCD 地图、等待初始位姿，在 NDT 模式对降采样 scan 匹配并维护最多 500 个 pose
的路径。`map_saver` 的传感器到 map 变换在源码中仍标为 TODO，使用它保存地图前必须验证
TF 假设。

## 3. Communication and Concurrency Design

Topic 被用于连续流（点云、位姿、路径、命令和状态）；当前项目没有自定义 action。KISS
的 `reset` 使用 service，因为它是一次性状态改变。

`ConeMapBuilder` 明确创建两个 `MutuallyExclusive` callback group：50 Hz pose 回调和较慢的
锥筒整合回调互不占用同一个组；要获得并发执行效果，部署者还需配合 MultiThreadedExecutor，
而其 `main` 当前使用普通 `rclcpp::spin`。其他项目节点未创建 callback group，按单执行器
回调模型运行。

时间设计：INS 使用 ground-truth header stamp；KISS 使用点云 stamp；EKF 用融合输出时间戳
发布 `odom -> base_link`。LiDAR 点云和 ConeArray 保留采样时间；`cones_viz` 在采样时刻
精确转换到 `map` 后发布。ConeMapBuilder 先按检测采样时间等待 TF，暂时不可用时排队重试，默认不使用 latest TF，
避免因为车辆运动把历史检测转换到错误位置。

## 4. Design Decisions

| 问题 | 选择 | 原因与替代方案 |
| --- | --- | --- |
| 真值赛道如何调试 | `/sim/lidar/track_cones` 静态、Transient Local marker | 与 `/mapping/cone_map_viz` 的估计地图隔离；替代方案是直接发布 ConeMap，但会把仿真真值伪装成 FSD 输出 |
| 坐标变换 | ConeMapBuilder 只按检测时间查询 TF，短暂缺失时排队重试 | 保持传感器时序，避免用车辆当前位姿转换历史点云造成系统性偏移 |
| Trackdrive 路径 | Delaunay 中点路径 | 可由局部锥筒地图恢复赛道中心；Skidpad/Acceleration 用已知几何以避免不必要的锥筒依赖 |
| Skidpad 交叉点跟踪 | Pure Pursuit 单调局部进度窗口 | 四圈与出口存在几何接近/重合点，按全部未来点搜索会直接跳至出口；每次只在有限连续窗口内推进可保证圈序 |
| Skidpad 横向前视 | 固定 `skidpad_lookahead=3.0 m` | 通用 `v×2.0` 在 5 m/s 时为 10 m，接近圆半径并跨越交叉点的曲率突变；短前视保留当前圆的转向至实际切换点 |
| 定位 TF 所有权 | EKF 是默认唯一动态 TF 发布者 | 防止 KISS、真值 bridge 与 EKF 同时发布 `base_link`；真值 bridge 仅以显式参数回退 |
| RViz marker 话题 | `.rviz` 使用 `Topic` 字段 | 无效字段会被 RViz 回退为 `visualization_marker_array`，该默认名不是系统接口 |
