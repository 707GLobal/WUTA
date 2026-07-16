"""Bridge simulator ground truth into the Level A WUTA-FSD interfaces."""

from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from wuta_msgs.msg import MissionState


class SimulationBridge(Node):
    """Adapt simulator truth to the interfaces needed by WUTA-FSD."""

    def __init__(self) -> None:
        super().__init__("simulation_bridge")

        self.declare_parameter("ground_truth_topic", "/sim/ground_truth")
        self.declare_parameter("publish_start_command", True)
        self.declare_parameter("publish_truth_localization", False)
        self.declare_parameter("manual_ready", False)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        ground_truth_topic = str(
            self.get_parameter("ground_truth_topic").value
        )
        self.publish_start_command = bool(
            self.get_parameter("publish_start_command").value
        )
        self.publish_truth_localization = bool(
            self.get_parameter("publish_truth_localization").value
        )
        self.manual_ready_enabled = bool(
            self.get_parameter("manual_ready").value
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.pose_pub = self.create_publisher(
            PoseStamped, "/localization/pose", 10
        )
        self.localization_ready_pub = self.create_publisher(
            Bool, "/system/localization_ready", 10
        )
        self.lidar_ready_pub = self.create_publisher(
            Bool, "/system/lidar_ready", 10
        )
        self.start_command_pub = self.create_publisher(
            Bool, "/system/start_command", 10
        )
        self.system_status_viz_pub = self.create_publisher(
            MarkerArray, "/system/status_viz", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.ground_truth_sub = self.create_subscription(
            Odometry, ground_truth_topic, self._on_ground_truth, 10
        )
        self.mission_state_sub = self.create_subscription(
            MissionState, "/system/mission_state", self._on_mission_state, 10
        )
        self.manual_ready_sub = self.create_subscription(
            PointStamped, "/clicked_point", self._on_manual_ready, 10
        )
        self.status_timer = self.create_timer(0.1, self._publish_status)
        self.received_ground_truth = False
        self.latest_mission_state: Optional[MissionState] = None
        self.latest_ground_truth: Optional[Odometry] = None
        self.manual_ready_confirmed = False

        self.get_logger().info(
            "Simulation bridge waiting for ground truth on %s; truth localization=%s"
            % (ground_truth_topic, self.publish_truth_localization)
        )

    def _on_ground_truth(self, msg: Odometry) -> None:
        self.received_ground_truth = True
        self.latest_ground_truth = msg

        if not self.publish_truth_localization:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.map_frame
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _on_mission_state(self, msg: MissionState) -> None:
        self.latest_mission_state = msg

    def _on_manual_ready(self, _msg: PointStamped) -> None:
        """Latch a manual-ready confirmation from RViz Publish Point."""
        if not self.manual_ready_enabled or self.manual_ready_confirmed:
            return
        self.manual_ready_confirmed = True
        self.get_logger().info(
            "Manual ready confirmed from RViz /clicked_point; publishing readiness"
        )

    def _publish_status(self) -> None:
        ready = (
            self.manual_ready_confirmed
            if self.manual_ready_enabled
            else self.received_ground_truth
        )
        lidar_ready = Bool()
        lidar_ready.data = ready
        self.lidar_ready_pub.publish(lidar_ready)

        if self.publish_truth_localization or self.manual_ready_enabled:
            localization_ready = Bool()
            localization_ready.data = ready
            self.localization_ready_pub.publish(localization_ready)

        if self.publish_start_command:
            start = Bool()
            start.data = True
            self.start_command_pub.publish(start)

        self._publish_status_visualization()

    def _publish_status_visualization(self) -> None:
        """Publish simulator runtime state as an RViz text marker."""
        mode_names = {
            MissionState.MISSION_TRACKDRIVE: "TRACKDRIVE",
            MissionState.MISSION_SKIDPAD: "SKIDPAD",
            MissionState.MISSION_ACCELERATION: "ACCELERATION",
        }
        current = self.latest_mission_state
        mission_state = (
            "FINISH" if current is not None and current.state == MissionState.FINISH
            else "EXPLORE" if current is not None and current.state == MissionState.EXPLORE
            else "READY" if current is not None and current.state == MissionState.READY
            else "IDLE"
        )
        mode_name = mode_names.get(
            current.mission_mode if current is not None else -1, "UNKNOWN"
        )
        completed = current is not None and current.state == MissionState.FINISH

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "simulator_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 2.5
        marker.scale.z = 0.45
        marker.color.r = 0.2 if completed else 1.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        lines = [
            "Mission: %s" % mode_name,
            "State: %s" % mission_state,
            "Complete: %s" % str(completed).lower(),
            "Ready: %s" % (
                "manual confirmed" if self.manual_ready_confirmed
                else "click RViz map" if self.manual_ready_enabled
                else "automatic"
            ),
        ]
        if self.latest_ground_truth is not None:
            position = self.latest_ground_truth.pose.pose.position
            velocity = self.latest_ground_truth.twist.twist.linear
            speed = (velocity.x * velocity.x + velocity.y * velocity.y) ** 0.5
            marker.pose.position.x = position.x + 1.0
            marker.pose.position.y = position.y + 1.0
            lines.extend(
                [
                    "GT speed: %.2f m/s" % speed,
                    "GT pose: (%.2f, %.2f) m" % (position.x, position.y),
                ]
            )
        else:
            lines.append("GT: waiting")
        marker.text = "\n".join(lines)

        markers = MarkerArray()
        markers.markers.append(marker)
        self.system_status_viz_pub.publish(markers)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node: Optional[SimulationBridge] = None
    try:
        node = SimulationBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
