"""Bring up the WUTA simulators in dependency order.

The individual simulators remain independent ROS 2 packages.  This file only
composes their launch files and starts the optional WUTA-FSD Level A pipeline
after its simulated inputs are available.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _delayed(period, *actions):
    return TimerAction(period=period, actions=list(actions))


def generate_launch_description():
    delay = LaunchConfiguration("startup_delay")
    launch_fsd = IfCondition(LaunchConfiguration("launch_fsd"))

    vehicle_share = FindPackageShare("vehicle_model")
    can_share = FindPackageShare("can_simulator")
    ins_share = FindPackageShare("ins_simulator")
    lidar_share = FindPackageShare("lidar_sim")
    localization_share = FindPackageShare("localization_manager")
    default_track = PathJoinSubstitution(
        [lidar_share, "tracks", "trackdrive.yaml"]
    )
    vehicle_start_x = PythonExpression(
        [
            "'-15.0' if '", LaunchConfiguration("mission_mode"),
            "' == 'skidpad' and '", LaunchConfiguration("start_x"),
            "' == 'auto' else ('0.0' if '", LaunchConfiguration("start_x"),
            "' == 'auto' else '", LaunchConfiguration("start_x"), "')",
        ]
    )

    # Stage 1: the source of truth.  vehicle_model depends at build/runtime on
    # autoware_msgs from WUTA-FSD and publishes /sim/ground_truth.
    vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [vehicle_share, "launch", "vehicle_model.launch.py"]
            )
        ),
        launch_arguments={
            "wheel_base": LaunchConfiguration("wheel_base"),
            "max_steer_angle": LaunchConfiguration("max_steer_angle"),
            "dt": LaunchConfiguration("vehicle_dt"),
            "start_x": vehicle_start_x,
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
        }.items(),
    )

    # Stage 2: all currently implemented sensors/feedback depend on ground
    # truth, so they are started only after the vehicle process has had time to
    # initialize.  Keep these as includes so each simulator stays standalone.
    can = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [can_share, "launch", "can_simulator.launch.py"]
            )
        )
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [lidar_share, "launch", "lidar_simulator.launch.py"]
            )
        ),
        launch_arguments={
            "track_file": LaunchConfiguration("track_file")
        }.items(),
    )

    # INS feeds the default KISS-ICP + EKF localization chain.  It remains
    # switchable for hardware-in-the-loop or truth-localization debugging.
    ins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [ins_share, "launch", "ins_simulator.launch.py"]
            )
        ),
        # Truth localization and estimated localization must never publish
        # competing map->base_link transforms. Selecting truth mode therefore
        # disables the INS branch automatically.
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("launch_ins"), "' == 'true' and '",
                LaunchConfiguration("use_ground_truth_localization"),
                "' == 'false'",
            ])
        ),
    )

    # The bridge always supplies simulator readiness and mission state.  Truth
    # pose/TF output is an explicit fallback; normally EKF owns those outputs.
    bridge = Node(
        package="simulator_bringup",
        executable="simulation_bridge",
        name="simulation_bridge",
        output="screen",
        parameters=[
            {
                "publish_mission_state": ParameterValue(
                    LaunchConfiguration("auto_start"), value_type=bool
                ),
                "mission_mode": LaunchConfiguration("mission_mode"),
                "publish_truth_localization": ParameterValue(
                    LaunchConfiguration("use_ground_truth_localization"),
                    value_type=bool,
                ),
            }
        ],
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [localization_share, "launch", "localization.launch.py"]
            )
        ),
        launch_arguments={
            "pointcloud_topic": "/hesai/pandar",
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("launch_localization"), "' == 'true' and '",
                LaunchConfiguration("use_ground_truth_localization"),
                "' == 'false'",
            ])
        ),
    )
    base_to_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "1.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "lidar",
        ],
        output="screen",
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # Stage 3: WUTA-FSD consumers, ordered by their topic data flow.
    lidar_detection = Node(
        package="lidar_detection",
        executable="lidar_detection_node",
        name="lidar_detection_node",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("lidar_detection"),
                    "config",
                    "lidar_detection.yaml",
                ]
            )
        ],
        output="screen",
        condition=launch_fsd,
    )
    cone_map_builder = Node(
        package="cone_map_builder",
        executable="cone_map_builder_node",
        name="cone_map_builder",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("cone_map_builder"),
                    "config",
                    "cone_map_builder.yaml",
                ]
            )
        ],
        output="screen",
        condition=launch_fsd,
    )
    boundary_detector = Node(
        package="boundary_detector",
        executable="boundary_detector_node",
        name="boundary_detector_node",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("boundary_detector"),
                    "config",
                    "boundary_detector.yaml",
                ]
            )
        ],
        output="screen",
        condition=launch_fsd,
    )
    path_generator = Node(
        package="path_generator",
        executable="path_generator_node",
        name="path_generator_node",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("path_generator"),
                    "config",
                    "path_generator.yaml",
                ]
            )
        ],
        output="screen",
        condition=launch_fsd,
    )
    controller = Node(
        package="controller",
        executable="controller_node",
        name="controller_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("controller"), "controller.yaml"]
            )
        ],
        output="screen",
        condition=launch_fsd,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "track_file",
                default_value=default_track,
                description="Track YAML path or installed track name.",
            ),
            DeclareLaunchArgument(
                "mission_mode",
                default_value="trackdrive",
                choices=["trackdrive", "skidpad", "acceleration"],
            ),
            DeclareLaunchArgument(
                "launch_fsd",
                default_value="true",
                choices=["true", "false"],
                description="Launch the WUTA-FSD perception-to-control chain.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Publish EXPLORE state for closed-loop simulation."
                ),
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="0.5",
                description="Seconds between dependency stages.",
            ),
            DeclareLaunchArgument(
                "launch_ins",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Launch ins_simulator to publish /cg410/odometry (default)."
                ),
            ),
            DeclareLaunchArgument(
                "launch_localization",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Launch KISS-ICP, EKF and localization_manager (default)."
                ),
            ),
            DeclareLaunchArgument(
                "use_ground_truth_localization",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Publish ground-truth /localization/pose and map->base_link "
                    "from simulation_bridge instead of the EKF output; this "
                    "automatically disables INS and localization."
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                choices=["true", "false"],
                description="Start RViz2 with the simulator visualization config.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("simulator_bringup"),
                        "rviz",
                        "wuta_simulator.rviz",
                    ]
                ),
                description="RViz2 config file.",
            ),
            DeclareLaunchArgument("wheel_base", default_value="1.53"),
            DeclareLaunchArgument("max_steer_angle", default_value="25.0"),
            DeclareLaunchArgument("vehicle_dt", default_value="0.02"),
            DeclareLaunchArgument(
                "start_x",
                default_value="auto",
                description=(
                    "Initial vehicle X coordinate. 'auto' selects -15 m for "
                    "skidpad (FSAC staging line) and 0 m for other missions."
                ),
            ),
            DeclareLaunchArgument("start_y", default_value="0.0"),
            DeclareLaunchArgument("start_yaw", default_value="0.0"),
            vehicle,
            _delayed(
                delay,
                bridge,
                can,
                ins,
                lidar,
                map_to_odom,
                base_to_lidar,
            ),
            _delayed(PythonExpression([delay, " * 2"]), localization, rviz),
            _delayed(PythonExpression([delay, " * 3"]), lidar_detection),
            _delayed(PythonExpression([delay, " * 4"]), cone_map_builder),
            _delayed(PythonExpression([delay, " * 5"]), boundary_detector),
            _delayed(PythonExpression([delay, " * 6"]), path_generator),
            _delayed(PythonExpression([delay, " * 7"]), controller),
        ]
    )
