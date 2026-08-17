#!/usr/bin/env python3
"""Drive successive Nav2 frontier goals while recording SLAM/cloud health.

This is a bounded validation utility, not an exploration behavior-tree plugin.
It deliberately selects only already-free cells at the free/unknown boundary.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quaternion_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    matrix = quaternion_matrix(x, y, z, w)
    return (
        math.atan2(matrix[2, 1], matrix[2, 2]),
        math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0))),
        math.atan2(matrix[1, 0], matrix[0, 0]),
    )


def fit_ground(points: np.ndarray, rng: random.Random) -> tuple[float, float, int, float]:
    """Return roll-like tilt, pitch-like tilt, inliers, and plane offset."""
    if len(points) > 3500:
        indices = np.asarray(rng.sample(range(len(points)), 3500), dtype=np.int64)
        points = points[indices]
    if len(points) < 100:
        return math.nan, math.nan, 0, math.nan

    best_mask = None
    best_count = 0
    for _ in range(120):
        selected = points[np.asarray(rng.sample(range(len(points)), 3), dtype=np.int64)]
        normal = np.cross(selected[1] - selected[0], selected[2] - selected[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1.0e-8:
            continue
        normal /= norm
        if abs(float(normal[2])) < 0.82:
            continue
        if normal[2] < 0.0:
            normal *= -1.0
        distance = np.abs(points @ normal - selected[0] @ normal)
        mask = distance < 0.045
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < 80:
        return math.nan, math.nan, best_count, math.nan

    inliers = points[best_mask]
    center = np.mean(inliers, axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal *= -1.0
    roll_tilt = math.degrees(math.atan2(float(normal[1]), float(normal[2])))
    pitch_tilt = math.degrees(math.atan2(float(-normal[0]), float(normal[2])))
    offset = -float(normal @ center)
    return roll_tilt, pitch_tilt, best_count, offset


class FrontierValidation(Node):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__("frontier_validation")
        self.arguments = arguments
        self.map_msg: OccupancyGrid | None = None
        self.global_costmap: OccupancyGrid | None = None
        self.last_cloud_time = 0.0
        self.rows: list[dict[str, object]] = []
        self.rng = random.Random(1088)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        map_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap, rclpy.qos.qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/lio/cloud_world", self._on_cloud, rclpy.qos.qos_profile_sensor_data
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_msg = message

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.global_costmap = message

    def _robot_transform(self):
        try:
            return self.tf_buffer.lookup_transform("map", "f4/base_link", rclpy.time.Time())
        except TransformException:
            return None

    def _on_cloud(self, message: PointCloud2) -> None:
        now = time.monotonic()
        if now - self.last_cloud_time < 1.0:
            return
        transform = self._robot_transform()
        if transform is None:
            return
        self.last_cloud_time = now
        try:
            values = point_cloud2.read_points_numpy(message, field_names=["x", "y", "z"], skip_nans=True)
        except (AssertionError, ValueError) as error:
            self.get_logger().warning(f"Cannot decode cloud: {error}")
            return
        points_map = np.asarray(values, dtype=np.float64).reshape((-1, 3))
        if len(points_map) > 12000:
            step = max(1, len(points_map) // 12000)
            points_map = points_map[::step]

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        origin = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        matrix = quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        points_base = (points_map - origin) @ matrix
        radius = np.hypot(points_base[:, 0], points_base[:, 1])
        usable = (radius > 0.6) & (radius < 7.0) & (points_base[:, 2] > -1.2) & (points_base[:, 2] < 0.15)
        base_ground = points_base[usable]
        map_ground = points_map[usable]
        map_roll, map_pitch, map_inliers, _ = fit_ground(map_ground, self.rng)
        base_roll, base_pitch, base_inliers, base_offset = fit_ground(base_ground, self.rng)
        pose_roll, pose_pitch, pose_yaw = quaternion_rpy(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        nearby = radius < 2.0
        low = nearby & (points_base[:, 2] > -0.65) & (points_base[:, 2] < -0.10)
        obstacle_band = nearby & (points_base[:, 2] >= -0.10) & (points_base[:, 2] < 0.80)
        map_unknown = map_free = map_occupied = 0
        cost_unknown = cost_free = cost_inflated = cost_lethal = 0
        for message, is_costmap in ((self.map_msg, False), (self.global_costmap, True)):
            if message is None:
                continue
            grid = self._grid_array(message)
            center_row, center_column = self._world_to_grid(message, translation.x, translation.y)
            cell_radius = int(math.ceil(2.0 / message.info.resolution))
            r0, r1 = max(0, center_row - cell_radius), min(grid.shape[0], center_row + cell_radius + 1)
            c0, c1 = max(0, center_column - cell_radius), min(grid.shape[1], center_column + cell_radius + 1)
            if r0 >= r1 or c0 >= c1:
                continue
            rows, columns = np.ogrid[r0:r1, c0:c1]
            circle = (rows - center_row) ** 2 + (columns - center_column) ** 2 <= cell_radius ** 2
            values = grid[r0:r1, c0:c1][circle]
            if is_costmap:
                cost_unknown = int(np.count_nonzero(values < 0))
                cost_free = int(np.count_nonzero(values == 0))
                # nav_msgs/OccupancyGrid costmap output is normalized to
                # 0..100 (the internal Costmap2D representation is 0..255).
                cost_inflated = int(np.count_nonzero((values > 0) & (values < 99)))
                cost_lethal = int(np.count_nonzero(values >= 99))
            else:
                map_unknown = int(np.count_nonzero(values < 0))
                map_free = int(np.count_nonzero(values == 0))
                map_occupied = int(np.count_nonzero(values >= 50))
        row = {
            "elapsed_s": round(now - self.started_monotonic, 3),
            "robot_x": round(translation.x, 4),
            "robot_y": round(translation.y, 4),
            "robot_z": round(translation.z, 4),
            "pose_roll_deg": round(math.degrees(pose_roll), 3),
            "pose_pitch_deg": round(math.degrees(pose_pitch), 3),
            "pose_yaw_deg": round(math.degrees(pose_yaw), 3),
            "map_ground_roll_deg": round(map_roll, 3),
            "map_ground_pitch_deg": round(map_pitch, 3),
            "base_ground_roll_deg": round(base_roll, 3),
            "base_ground_pitch_deg": round(base_pitch, 3),
            "base_ground_offset_m": round(base_offset, 4),
            "map_ground_inliers": map_inliers,
            "base_ground_inliers": base_inliers,
            "near_low_points": int(np.count_nonzero(low)),
            "near_obstacle_band_points": int(np.count_nonzero(obstacle_band)),
            "map_2m_unknown": map_unknown,
            "map_2m_free": map_free,
            "map_2m_occupied": map_occupied,
            "cost_2m_unknown": cost_unknown,
            "cost_2m_free": cost_free,
            "cost_2m_inflated": cost_inflated,
            "cost_2m_lethal": cost_lethal,
        }
        self.rows.append(row)
        if len(self.rows) % 5 == 0:
            self.get_logger().info(
                "cloud pose_rp=(%.1f,%.1f) map_ground_rp=(%.1f,%.1f) "
                "base_ground_rp=(%.1f,%.1f) low/obstacle=%d/%d"
                % (
                    row["pose_roll_deg"], row["pose_pitch_deg"], row["map_ground_roll_deg"],
                    row["map_ground_pitch_deg"], row["base_ground_roll_deg"],
                    row["base_ground_pitch_deg"], row["near_low_points"],
                    row["near_obstacle_band_points"],
                )
            )

    @staticmethod
    def _grid_array(message: OccupancyGrid) -> np.ndarray:
        return np.asarray(message.data, dtype=np.int16).reshape((message.info.height, message.info.width))

    @staticmethod
    def _world_to_grid(message: OccupancyGrid, x: float, y: float) -> tuple[int, int]:
        column = int(math.floor((x - message.info.origin.position.x) / message.info.resolution))
        row = int(math.floor((y - message.info.origin.position.y) / message.info.resolution))
        return row, column

    def _cost_at(self, x: float, y: float) -> int:
        if self.global_costmap is None:
            return 255
        row, column = self._world_to_grid(self.global_costmap, x, y)
        if row < 0 or column < 0 or row >= self.global_costmap.info.height or column >= self.global_costmap.info.width:
            return 255
        return int(self.global_costmap.data[row * self.global_costmap.info.width + column])

    def _line_is_free(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        if self.map_msg is None:
            return False
        grid = self._grid_array(self.map_msg)
        distance = math.hypot(end_x - start_x, end_y - start_y)
        steps = max(1, int(math.ceil(distance / (0.5 * self.map_msg.info.resolution))))
        for step in range(1, steps + 1):
            fraction = step / steps
            x = start_x + fraction * (end_x - start_x)
            y = start_y + fraction * (end_y - start_y)
            row, column = self._world_to_grid(self.map_msg, x, y)
            if row < 0 or column < 0 or row >= grid.shape[0] or column >= grid.shape[1]:
                return False
            if grid[row, column] != 0:
                return False
            cost = self._cost_at(x, y)
            # /global_costmap/costmap is nav_msgs/OccupancyGrid: unknown is
            # represented as -1 (not Costmap2D's internal uint8 value 255).
            if cost < 0 or cost >= 100:
                return False
        return True

    def choose_frontier(self, excluded: list[tuple[float, float]]) -> tuple[float, float] | None:
        if self.map_msg is None:
            return None
        transform = self._robot_transform()
        if transform is None:
            return None
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        grid = self._grid_array(self.map_msg)
        free = grid == 0
        unknown = grid < 0
        adjacent_unknown = np.zeros_like(unknown)
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        frontier_rows, frontier_columns = np.nonzero(free & adjacent_unknown)
        if len(frontier_rows) == 0:
            return None

        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        order = np.arange(len(frontier_rows))
        self.rng.shuffle(order)
        candidates: list[tuple[float, float, float]] = []
        rejected_distance = 0
        rejected_map_clearance = 0
        rejected_cost = 0
        rejected_history = 0
        clearance = max(2, int(math.ceil(0.40 / resolution)))
        for index in order[: min(8000, len(order))]:
            row = int(frontier_rows[index])
            column = int(frontier_columns[index])
            frontier_x = origin_x + (column + 0.5) * resolution
            frontier_y = origin_y + (row + 0.5) * resolution
            frontier_distance = math.hypot(frontier_x - robot_x, frontier_y - robot_y)
            if frontier_distance < self.arguments.min_goal_distance or frontier_distance > self.arguments.max_goal_distance:
                rejected_distance += 1
                continue
            # Stop on the known side of the frontier. The exact boundary cell
            # often receives inflation from a neighboring obstacle endpoint.
            inset = min(0.65, max(0.0, frontier_distance - self.arguments.min_goal_distance))
            x = frontier_x - inset * (frontier_x - robot_x) / frontier_distance
            y = frontier_y - inset * (frontier_y - robot_y) / frontier_distance
            distance = math.hypot(x - robot_x, y - robot_y)
            row, column = self._world_to_grid(self.map_msg, x, y)
            if row < 0 or column < 0 or row >= grid.shape[0] or column >= grid.shape[1] or grid[row, column] != 0:
                rejected_map_clearance += 1
                continue
            r0, r1 = max(0, row - clearance), min(grid.shape[0], row + clearance + 1)
            c0, c1 = max(0, column - clearance), min(grid.shape[1], column + clearance + 1)
            if np.any(grid[r0:r1, c0:c1] >= 50):
                rejected_map_clearance += 1
                continue
            # A just-expanded StaticLayer can report this boundary cell as
            # unknown (-1) for one update cycle. Wait for the costmap update;
            # the validator must not select an unplannable test goal.
            cost = self._cost_at(x, y)
            if cost < 0 or cost >= 80:
                rejected_cost += 1
                continue
            if not self._line_is_free(robot_x, robot_y, x, y):
                rejected_cost += 1
                continue
            if any(math.hypot(x - old_x, y - old_y) < 1.0 for old_x, old_y in excluded):
                rejected_history += 1
                continue
            known_gain = int(np.count_nonzero(unknown[max(0, row - 10):row + 11, max(0, column - 10):column + 11]))
            candidates.append((distance + 0.01 * known_gain, x, y))
        if not candidates:
            self.get_logger().warning(
                "Frontier filters: total=%d distance=%d map_clearance=%d cost=%d history=%d"
                % (
                    len(frontier_rows), rejected_distance, rejected_map_clearance,
                    rejected_cost, rejected_history,
                )
            )
            # Sparse ray maps can have a jagged frontier whose boundary cells
            # are all inflated. Fall back to the farthest safe known-free cell
            # that still has unknown space nearby; driving there produces the
            # next scan from which a clean frontier can be selected.
            free_rows, free_columns = np.nonzero(free)
            fallback_order = np.arange(len(free_rows))
            self.rng.shuffle(fallback_order)
            for index in fallback_order[: min(12000, len(fallback_order))]:
                row = int(free_rows[index])
                column = int(free_columns[index])
                x = origin_x + (column + 0.5) * resolution
                y = origin_y + (row + 0.5) * resolution
                distance = math.hypot(x - robot_x, y - robot_y)
                if distance < self.arguments.min_goal_distance or distance > self.arguments.max_goal_distance:
                    continue
                r0, r1 = max(0, row - clearance), min(grid.shape[0], row + clearance + 1)
                c0, c1 = max(0, column - clearance), min(grid.shape[1], column + clearance + 1)
                if np.any(grid[r0:r1, c0:c1] >= 50):
                    continue
                cost = self._cost_at(x, y)
                if cost < 0 or cost >= 80:
                    continue
                if not self._line_is_free(robot_x, robot_y, x, y):
                    continue
                if any(math.hypot(x - old_x, y - old_y) < 1.0 for old_x, old_y in excluded):
                    continue
                gain_radius = max(clearance + 1, int(math.ceil(1.5 / resolution)))
                unknown_gain = int(
                    np.count_nonzero(
                        unknown[
                            max(0, row - gain_radius):row + gain_radius + 1,
                            max(0, column - gain_radius):column + gain_radius + 1,
                        ]
                    )
                )
                candidates.append((distance + 0.002 * unknown_gain, x, y))
            if candidates:
                self.get_logger().info("Using safe known-free staging point near frontier")
            else:
                return None
        _, x, y = max(candidates)
        return x, y

    def send_goal(self, x: float, y: float) -> tuple[bool, str]:
        transform = self._robot_transform()
        if transform is None:
            return False, "no_tf"
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        yaw = math.atan2(y - robot_y, x - robot_x)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            return False, "send_timeout"
        handle = future.result()
        if not handle.accepted:
            return False, "rejected"
        result = handle.get_result_async()
        deadline = time.monotonic() + self.arguments.goal_timeout
        while rclpy.ok() and not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
        if not result.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            return False, "timeout"
        wrapped = result.result()
        return wrapped.status == GoalStatus.STATUS_SUCCEEDED, f"status_{wrapped.status}"

    def run(self) -> int:
        self.started_monotonic = time.monotonic()
        self.get_logger().info("Waiting for /map, TF, and NavigateToPose...")
        deadline = time.monotonic() + self.arguments.startup_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if self.map_msg is not None and self._robot_transform() is not None and self.action_client.server_is_ready():
                break
        else:
            self.get_logger().error("Stack did not become ready")
            return 2

        settle_deadline = time.monotonic() + self.arguments.settle_time
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.10)

        attempted: list[tuple[float, float]] = []
        successes = 0
        for goal_index in range(self.arguments.goals):
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.10)
            target = self.choose_frontier(attempted)
            if target is None:
                self.get_logger().warning("No safe frontier candidate remains")
                break
            attempted.append(target)
            self.get_logger().info(
                f"Frontier {goal_index + 1}/{self.arguments.goals}: ({target[0]:.2f}, {target[1]:.2f})"
            )
            succeeded, status = self.send_goal(*target)
            successes += int(succeeded)
            self.get_logger().info(f"Frontier result: {status}")

        output = Path(self.arguments.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        if self.rows:
            with output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)
        self.get_logger().info(
            f"Validation complete: {successes}/{len(attempted)} goals succeeded, "
            f"{len(self.rows)} cloud samples -> {output}"
        )
        return 0 if attempted and successes == len(attempted) else 1


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--goals", type=int, default=6)
    parser.add_argument("--startup-timeout", type=float, default=120.0)
    parser.add_argument("--goal-timeout", type=float, default=90.0)
    parser.add_argument("--settle-time", type=float, default=2.0)
    parser.add_argument("--min-goal-distance", type=float, default=1.3)
    parser.add_argument("--max-goal-distance", type=float, default=4.0)
    parser.add_argument("--output", default="log/runtime/frontier_validation.csv")
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    rclpy.init()
    node = FrontierValidation(arguments)
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
