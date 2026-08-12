#!/usr/bin/python3
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalPoseBridge(Node):
    """Translate RViz's standard /goal_pose topic into NavigateToPose goals."""

    def __init__(self):
        super().__init__("nav2_goal_pose_bridge")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("default_frame", "map")
        self._action_name = str(self.get_parameter("action_name").value)
        self._default_frame = str(self.get_parameter("default_frame").value)
        self._client = ActionClient(self, NavigateToPose, self._action_name)
        self._goal_handle = None
        self._last_feedback_log_ns = 0
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self._on_goal_pose,
            10,
        )
        self.get_logger().info(
            f"RViz goal bridge ready: {self.get_parameter('goal_topic').value} -> {self._action_name}"
        )

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        if not pose.header.frame_id:
            pose.header.frame_id = self._default_frame
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            pose.header.stamp = self.get_clock().now().to_msg()
        if not self._client.server_is_ready():
            self.get_logger().error(f"Nav2 action server {self._action_name} is not active yet")
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        x = pose.pose.position.x
        y = pose.pose.position.y
        q = pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.get_logger().info(f"Sending Nav2 goal: frame={pose.header.frame_id} x={x:.3f} y={y:.3f} yaw={yaw:.1f}deg")
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"NavigateToPose request failed: {exc}")
            return
        if not handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected")
            return
        self._goal_handle = handle
        self.get_logger().info("NavigateToPose goal accepted")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_log_ns < 2_000_000_000:
            return
        self._last_feedback_log_ns = now_ns
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Nav2 remaining={feedback.distance_remaining:.2f}m "
            f"recoveries={feedback.number_of_recoveries}"
        )

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().error(f"NavigateToPose result failed: {exc}")
            self._goal_handle = None
            return
        labels = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        label = labels.get(wrapped.status, f"STATUS_{wrapped.status}")
        log = self.get_logger().info if wrapped.status == GoalStatus.STATUS_SUCCEEDED else self.get_logger().warning
        log(f"NavigateToPose finished: {label}")
        self._goal_handle = None


def main() -> None:
    rclpy.init()
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
