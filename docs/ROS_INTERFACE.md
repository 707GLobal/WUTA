# ROS 2 接口规范

> 接口名和类型均来自节点源码。除特别说明外，`create_publisher(..., 10)`/
> `create_subscription(..., 10)` 表示 depth 10、可靠、volatile 的默认 QoS；
> `SensorDataQoS` 表示 best-effort、volatile、keep-last 的传感器 QoS。
>
> **默认定位链：** `ins_simulator`、KISS-ICP、EKF 与 localization_manager 默认启动。
> `/sim/ground_truth` → `/cg410/odometry` 与 `/hesai/pandar` → `/kiss/odometry` 由 EKF
> 融合为 `/odometry/filtered`，再转换为 `/localization/pose`。

## 1. Topic Interface

| Topic | Type | Publisher | Subscriber | 频率 / QoS |
| --- | --- | --- | --- | --- |
| `/sim/ground_truth` | `nav_msgs/msg/Odometry` | `vehicle_model` | lidar/can/bridge | vehicle `dt`，默认 50 Hz；depth 50 |
| `/hesai/pandar` | `sensor_msgs/msg/PointCloud2` | `lidar_simulator` | lidar_detection、NDT、map_saver、KISS-ICP | 默认 10 Hz；发布 depth 10；检测/NDT/map_saver 用 SensorDataQoS |
| `/sim/lidar/visible_cones` | `visualization_msgs/msg/MarkerArray` | `lidar_simulator` | RViz | 随扫描；depth 10；`lidar` frame；marker stamp=0（最新 TF） |
| `/sim/lidar/track_cones` | `visualization_msgs/msg/MarkerArray` | `lidar_simulator` | RViz | 启动时一次；Reliable + Transient Local；`map` frame |
| `/localization/velocity` | `geometry_msgs/msg/TwistStamped` | `can_simulator` | controller | 随 ground truth；depth 50 |
| `/cg410/odometry` | `nav_msgs/msg/Odometry` | `ins_simulator` | `ekf_node` | 默认启动，20 Hz；depth 20；`map` frame |
| `/localization/pose` | `geometry_msgs/msg/PoseStamped` | `localization_manager`（默认）或 simulation_bridge（真值回退） | 建图/规划/控制 | 随 EKF 输出；depth 10 |
| `/perception/lidar/cones` | `wuta_msgs/msg/ConeArray` | lidar_detection | cone_map_builder | 随点云；depth 10 |
| `/perception/lidar/cones_viz` | `visualization_msgs/msg/MarkerArray` | lidar_detection | RViz | 有订阅者时；转换到 `map` 后发布；使用采样时间；depth 10 |
| `/mapping/cone_map` | `wuta_msgs/msg/ConeMap` | cone_map_builder | boundary_detector、mission_manager | 5 Hz 定时器；depth 10 |
| `/mapping/cone_map_viz` | `visualization_msgs/msg/MarkerArray` | cone_map_builder | RViz | 5 Hz；depth 10 |
| `/planning/centerline` | `autoware_msgs/msg/Lane` | boundary_detector | path_generator | 收到地图时；depth 10 |
| `/planning/centerline_viz` | `visualization_msgs/msg/MarkerArray` | boundary_detector | RViz | 有订阅者时；depth 10 |
| `/planning/final_waypoints` | `autoware_msgs/msg/Lane` | path_generator | controller | 中心线或任务状态触发；depth 10 |
| `/control/command` | `autoware_msgs/msg/Command` | controller | vehicle_model | 控制定时器，默认 50 Hz；depth 10 |
| `/system/mission_complete` | `std_msgs/msg/Bool` | controller | simulation_bridge | Skidpad 在固定 25 m 出口终点停车后一次发布 `true`；depth 10 |
| `/control/target_viz` | `visualization_msgs/msg/MarkerArray` | controller | RViz | 有订阅者时；depth 10 |
| `/system/mission_state` | `wuta_msgs/msg/MissionState` | simulation_bridge（默认）或 mission_manager | 规划/控制/定位/NDT/map_saver | bridge 10 Hz；depth 10 |
| `/system/lidar_ready` | `std_msgs/msg/Bool` | simulation_bridge | mission_manager | 10 Hz；depth 10 |
| `/system/localization_ready` | `std_msgs/msg/Bool` | localization_manager（默认）或 simulation_bridge（真值回退） | mission_manager | 随定位输出；depth 10 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | robot_localization `ekf_node` | localization_manager | 默认融合输出；50 Hz；`odom` frame |
| `/kiss/odometry` | `nav_msgs/msg/Odometry` | `kiss_icp_node` | `ekf_node`、map_saver | 默认约 10 Hz；`odom` frame；KISS 不发布 TF |
| `/ndt/pose` | `geometry_msgs/msg/PoseStamped` | ndt_localization | localization_manager | NDT 激活时；depth 10 |
| `/ndt/path` | `nav_msgs/msg/Path` | ndt_localization | 工具/RViz | NDT 激活时；depth 10 |
| `/ndt/aligned_cloud` | `sensor_msgs/msg/PointCloud2` | ndt_localization | 工具/RViz | 有订阅者时；depth 10 |
| `/ndt/map_ready` | `std_msgs/msg/Bool` | map_saver | 外部编排 | 保存成功时；depth 10 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 外部（RViz/定位工具） | ndt_localization | depth 10 |
| `/system/emergency` | `std_msgs/msg/Bool` | 外部 | mission_manager | depth 10 |
| `/system/mission_mode_cmd` | `std_msgs/msg/String` | 外部 | mission_manager | depth 10 |
| `/system/inspection_trigger` | `std_msgs/msg/Bool` | 外部 | mission_manager | depth 10 |
| `/system/inspection_result` | `std_msgs/msg/String` | mission_manager | 外部 | 车检触发后；当前内容为未实现提示 |

KISS-ICP 在 `publish_debug_clouds=true` 时还会发布相对名称 `kiss/frame`、
`kiss/keypoints`、`kiss/local_map`（均 `PointCloud2`）；默认配置关闭这些调试点云。

### Message Structure

```text
wuta_msgs/msg/Cone
  geometry_msgs/Point position
  uint8 color  # UNKNOWN=0, BLUE=1, YELLOW=2, ORANGE=3
  float32 confidence

wuta_msgs/msg/ConeArray
  std_msgs/Header header  # sensor 或 map frame
  Cone[] cones

wuta_msgs/msg/ConeMap
  std_msgs/Header header  # map
  Cone[] blue_cones, yellow_cones, orange_cones, unknown_cones
  bool is_closed

wuta_msgs/msg/MissionState
  Header header; uint8 state; uint8 mission_mode; uint8 localization_mode
  string description

autoware_msgs/msg/Lane
  Header header
  Waypoint[] waypoints  # PoseStamped pose + TwistStamped twist

autoware_msgs/msg/Command
  float64 speed
  float64 angle
  int32 dv_state
```

## 2. Service and Action Interface

本项目自身节点未定义 `.srv` 或 `.action`。默认 bringup 中的 KISS-ICP 节点创建 reset
service。

作为源码依赖引入的 KISS-ICP ROS 节点创建相对名 `reset` service：

| Service | Type | Request | Response | 作用 |
| --- | --- | --- | --- | --- |
| `/kiss/reset`（kiss_icp_node） | `std_srvs/srv/Empty` | 空 | 空 | 重置 KISS-ICP 状态 |

仓库中的 robot_localization 包定义以下服务类型。它们由该第三方包的过滤/地理坐标节点
按自身配置提供，不由 WUTA 的 `simulator.launch.py` 启动，因此不能视为默认系统服务。

| Type | Request | Response |
| --- | --- | --- |
| `robot_localization/srv/FromLL` | `geographic_msgs/GeoPoint ll_point` | `geometry_msgs/Point map_point` |
| `robot_localization/srv/ToLL` | `geometry_msgs/Point map_point` | `geographic_msgs/GeoPoint ll_point` |
| `robot_localization/srv/SetDatum` | `geographic_msgs/GeoPose geo_pose` | 空 |
| `robot_localization/srv/SetPose` | `geometry_msgs/PoseWithCovarianceStamped pose` | 空 |
| `robot_localization/srv/GetState` | `builtin_interfaces/Time time_stamp`、`string frame_id` | `float64[15] state`、`float64[225] covariance` |
| `robot_localization/srv/ToggleFilterProcessing` | `bool on` | `bool status` |

仓库中未定义 action 文件。

## 3. TF Frame

默认仿真 TF 树：

```text
map
 └─ odom            static: simulator.launch.py，仿真中与 map 同原点
     └─ base_link   dynamic: ekf_node
         └─ lidar   static: simulator.launch.py，平移 (0, 0, 1) m
```

KISS-ICP 的 `lidar_odom_frame=odom`、`base_frame=base_link`，且
`publish_odom_tf=false`，避免与 EKF 竞争 TF。EKF 配置为 `world_frame=odom`，发布唯一的
动态 `odom -> base_link`。`use_ground_truth_localization:=true` 时，simulation_bridge
才会额外发布真值 `map -> base_link`，因此不能和默认 EKF TF 同时用于 FSD。

`/hesai/pandar` 与 `/perception/lidar/cones` 保留 ground-truth 采样时间，供感知和建图
使用。`/perception/lidar/cones_viz` 在采样时刻精确转换到 `map` 后发布，因此 RViz 不再
需要查询历史 `map -> lidar` TF。仅用于 RViz 的
`/sim/lidar/visible_cones` 使用零时间戳请求最新 `map -> odom -> base_link -> lidar` TF。
`/sim/lidar/track_cones` 与 `/mapping/cone_map_viz` 直接在 `map`。

## 4. Parameters

| Node | 参数（类型） | 来源 / 说明 |
| --- | --- | --- |
| vehicle_model | `wheel_base`、`max_steer_angle`、`dt`、`start_x/y/yaw`（double） | `vehicle_model.py` / launch |
| lidar_simulator | topic/frame 名（string）、`publish_rate_hz`/FOV/范围/噪声（double）、点数（int）、开关（bool） | `config/lidar_simulator.yaml` |
| simulation_bridge | `ground_truth_topic`、`mission_mode`、`map_frame`、`base_frame`（string）；`publish_mission_state`、`publish_truth_localization`（bool） | `simulation_bridge.py`；接收 `/system/mission_complete=true` 后发布 `FINISH` |
| lidar_detection_node | `detector_type`、topic 名、地面/体素/聚类/几何阈值、`model_path` | `config/lidar_detection.yaml` |
| cone_map_builder | `merge_distance`、`min_hit_count`、闭环阈值、`assign_colors`、`map_save_path`、`tf_lookup_timeout_sec`、`pending_detection_timeout_sec`、`max_pending_detections`、`use_latest_tf_fallback` | `config/cone_map_builder.yaml`；默认只使用检测采样时刻 TF，缺失时排队重试 |
| boundary_detector_node | `lookahead_distance`、`desired_velocity` | `config/boundary_detector.yaml` |
| path_generator_node | Trackdrive/Skidpad/Acceleration 速度、半径、点数、长度；Skidpad map 参考、出口和制动距离 | `config/path_generator.yaml` |
| controller_node | 车辆几何、Pure Pursuit lookahead/连续进度窗口、`skidpad_lookahead=3.0 m`、`control_rate_hz`、Skidpad 完成位置/速度阈值 | `config/controller.yaml`；仅 `MISSION_SKIDPAD` 使用固定前视，其它模式仍使用动态前视 |
| mission_manager | `mission_mode`（string） | `mission_manager.cpp` |
| localization_manager | 无显式声明参数 | 默认定位集成；通过固定话题与 MissionState 选源 |
| ndt_localization / map_saver | 地图路径、NDT/体素参数、累积距离 | `config/ndt_localization.yaml` |
| kiss_icp_node | frame/TF、协方差、范围、体素、阈值、迭代参数 | `kiss_icp_wrapper/config/kiss_icp_hesai128.yaml` |

完整参数名、默认值与类型以对应 YAML 和节点 `declare_parameter` 为准。
