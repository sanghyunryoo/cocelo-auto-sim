#!/usr/bin/env python3
"""Low-rate terminal display for front-wall angle and Super-LIO state."""

import argparse
import math
import sys
import time

import rclpy
from autonomy_light.msg import WallAngle
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


def positive_rate(value: str) -> float:
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0.0:
        raise argparse.ArgumentTypeError("rate must be a finite value greater than zero")
    return rate


class WallStateMonitor(Node):
    _TABLE_WIDTH = 96
    _REASONS = {
        WallAngle.REASON_DETECTED: "MEASUREMENT ACCEPTED",
        WallAngle.REASON_INVALID_CLOUD: "INPUT CLOUD HAS NO FRAME OR XYZ",
        WallAngle.REASON_TF_UNAVAILABLE: "MAP TO BASE TF UNAVAILABLE",
        WallAngle.REASON_CLOUD_READ_ERROR: "POINTCLOUD READ ERROR",
        WallAngle.REASON_INSUFFICIENT_ROI_POINTS: "TOO FEW ROI POINTS",
        WallAngle.REASON_NO_QUALIFIED_PLANE: "NO PLANE PASSED WALL GATES",
        WallAngle.REASON_INITIALIZING: "STARTUP ALIGNMENT REQUIRED",
    }
    _SOURCES = {
        WallAngle.SOURCE_NONE: "NONE",
        WallAngle.SOURCE_RAW: "RAW",
        WallAngle.SOURCE_SUBMAP: "SUBMAP",
        WallAngle.SOURCE_FUSED: "FUSED",
        WallAngle.SOURCE_RAW_CONFLICT: "RAW (WALL CHANGE)",
    }

    _RESET = "\033[0m"
    _BOLD = "\033[1m"
    _DIM = "\033[2m"
    _RED = "\033[31m"
    _GREEN = "\033[32m"
    _YELLOW = "\033[33m"
    _BLUE = "\033[34m"
    _CYAN = "\033[36m"

    def __init__(self, wall_topic: str, odom_topic: str, rate_hz: float,
                 color_mode: str, init_only: bool = False) -> None:
        super().__init__("wall_state_monitor")
        self._color = color_mode == "always" or (
            color_mode == "auto" and sys.stdout.isatty())
        self._redraw = sys.stdout.isatty()
        self._wall: WallAngle | None = None
        self._wall_received_at: float | None = None
        self._odom: Odometry | None = None
        self._odom_received_at: float | None = None
        self._init_only = init_only
        self._initialization_accepted = False
        self.create_subscription(WallAngle, wall_topic, self._on_wall, 10)
        if not init_only:
            self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_timer(1.0 / rate_hz, self._print_state)
        self.get_logger().info(
            f"Monitoring wall={wall_topic}, odom={odom_topic} at {rate_hz:g} Hz")

    def _on_wall(self, message: WallAngle) -> None:
        self._wall = message
        self._wall_received_at = time.monotonic()
        if self._init_only and message.initialization_complete:
            self._initialization_accepted = True

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._odom_received_at = time.monotonic()

    @staticmethod
    def _age(received_at: float | None) -> str:
        if received_at is None:
            return "n/a"
        return f"{time.monotonic() - received_at:.2f}s"

    @staticmethod
    def _roll_pitch_yaw_deg(odom: Odometry) -> tuple[float, float, float]:
        quaternion = odom.pose.pose.orientation
        sin_roll = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cos_roll = 1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2)
        roll = math.atan2(sin_roll, cos_roll)

        sin_pitch = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

        sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    def _paint(self, text: str, color: str = "", bold: bool = False) -> str:
        if not self._color:
            return text
        prefix = (self._BOLD if bold else "") + color
        return f"{prefix}{text}{self._RESET}" if prefix else text

    def _measurement_text(self, angle_deg: float, distance_m: float,
                          sigma_deg: float, inliers: int) -> str:
        return (f"angle {angle_deg:+.2f} deg | range {distance_m:.3f} m | "
                f"sigma {sigma_deg:.2f} deg | inliers {inliers}")

    def _initialization_rows(self) -> list[tuple[str, str, str, bool]]:
        if self._wall is None:
            return [
                ("INIT", "LOCKED | waiting for qualified front-wall measurement", self._RED, True),
                ("TARGET", "mean wall angle target 0.00 +/-1.00 deg", self._RED, False),
                ("SLAM", "OPERATIONAL SLAM NOT STARTED", self._RED, True),
            ]
        if self._wall.initialization_complete:
            angle = (f" | mean wall angle "
                     f"{self._wall.initialization_angle_error_deg:+.2f} deg"
                     if math.isfinite(self._wall.initialization_angle_error_deg)
                     else "")
            return [
                ("INIT", f"ACCEPTED{angle}", self._GREEN, True),
                ("SLAM", "STARTING OPERATIONAL SLAM WITH FRESH MAP/YAW", self._GREEN, True),
            ]
        if math.isfinite(self._wall.initialization_angle_error_deg):
            mean_text = f"mean wall angle {self._wall.initialization_angle_error_deg:+.2f} deg"
        else:
            mean_text = "waiting for qualified front-wall fit"
        return [
            ("INIT", "LOCKED | align robot before SLAM starts", self._RED, True),
            ("MEAN", mean_text, self._RED, False),
            ("TARGET", f"0.00 +/-{self._wall.initialization_tolerance_deg:.2f} deg",
             self._RED, False),
            ("FRAMES", f"valid fits {self._wall.initialization_sample_count}/"
             f"{self._wall.initialization_required_sample_count}", self._RED, False),
            ("SLAM", "OPERATIONAL SLAM NOT STARTED", self._RED, True),
        ]

    def _wall_rows(self) -> list[tuple[str, str, str, bool]]:
        if self._wall is None:
            return [
                ("WALL", "WAITING FOR WallAngle MESSAGE", self._YELLOW, True),
                ("RAW", "waiting for current-scan fit", self._YELLOW, False),
                ("SUBMAP", "waiting for prior scans", self._BLUE, False),
                ("FINAL", "waiting for fusion output", self._YELLOW, False),
                ("QUALITY", "", "", False),
            ]

        raw_reason = self._REASONS.get(
            self._wall.raw_detection_reason,
            f"UNKNOWN REASON ({self._wall.raw_detection_reason})")
        if self._wall.raw_detected and math.isfinite(self._wall.raw_relative_angle_deg):
            raw_text = self._measurement_text(
                self._wall.raw_relative_angle_deg, self._wall.raw_distance_m,
                self._wall.raw_angle_stddev_deg, self._wall.raw_inlier_count)
            raw_color = self._GREEN
        else:
            raw_text = f"NOT DETECTED | {raw_reason}"
            raw_color = self._YELLOW

        if self._wall.submap_detected and math.isfinite(
                self._wall.submap_relative_angle_deg):
            submap_text = self._measurement_text(
                self._wall.submap_relative_angle_deg, self._wall.submap_distance_m,
                self._wall.submap_angle_stddev_deg, self._wall.submap_inlier_count)
            submap_text += f" | cached ROI {self._wall.submap_point_count}"
            submap_color = self._BLUE
        else:
            submap_text = f"NOT READY | cached ROI {self._wall.submap_point_count}"
            submap_color = self._DIM

        final_reason = self._REASONS.get(
            self._wall.detection_reason,
            f"UNKNOWN REASON ({self._wall.detection_reason})")
        if self._wall.initialization_complete:
            init_error = (f" | initial mean wall angle "
                          f"{self._wall.initialization_angle_error_deg:+.2f} deg"
                          if math.isfinite(self._wall.initialization_angle_error_deg)
                          else "")
            init_text = f"COMPLETE{init_error}"
            init_color = self._GREEN
        elif math.isfinite(self._wall.initialization_angle_error_deg):
            init_text = (
                f"LOCKED | mean wall angle {self._wall.initialization_angle_error_deg:+.2f} deg | "
                f"target 0.00 +/-{self._wall.initialization_tolerance_deg:.2f} deg | "
                f"frames {self._wall.initialization_sample_count}/"
                f"{self._wall.initialization_required_sample_count}")
            init_color = self._RED
        else:
            init_text = (
                f"LOCKED | waiting for qualified wall | frames "
                f"{self._wall.initialization_sample_count}/"
                f"{self._wall.initialization_required_sample_count} | target 0.00 "
                f"+/-{self._wall.initialization_tolerance_deg:.2f} deg")
            init_color = self._RED
        if self._wall.detected and math.isfinite(self._wall.relative_angle_deg):
            source = self._SOURCES.get(
                self._wall.estimate_source,
                f"UNKNOWN SOURCE ({self._wall.estimate_source})")
            final_text = self._measurement_text(
                self._wall.relative_angle_deg, self._wall.distance_m,
                self._wall.angle_stddev_deg, self._wall.inlier_count)
            final_text += f" | source {source}"
            final_color = (self._YELLOW if
                           self._wall.estimate_source == WallAngle.SOURCE_RAW_CONFLICT
                           else self._GREEN)
            wall_text = f"DETECTED | message age {self._age(self._wall_received_at)}"
            wall_color = self._GREEN
        else:
            final_text = f"NOT DETECTED | {final_reason}"
            final_color = self._RED if self._wall.detection_reason in (
                WallAngle.REASON_INVALID_CLOUD, WallAngle.REASON_TF_UNAVAILABLE,
                WallAngle.REASON_CLOUD_READ_ERROR, WallAngle.REASON_INITIALIZING) else self._YELLOW
            wall_text = (f"NOT DETECTED | message age {self._age(self._wall_received_at)}")
            wall_color = final_color

        quality = (
            f"roi {self._wall.roi_point_count} | "
            f"best raw candidate {self._wall.best_candidate_inlier_count} | "
            f"required {self._wall.required_inlier_count}")
        rows = [
            ("WALL", wall_text, wall_color, True),
            ("RAW", raw_text, raw_color, False),
            ("SUBMAP", submap_text, submap_color, False),
            ("FINAL", final_text, final_color, True),
            ("QUALITY", quality, "", False),
        ]
        if not self._wall.initialization_complete:
            rows.insert(3, ("INIT", init_text, init_color, True))
        return rows

    def _fit(self, text: str, width: int) -> str:
        if len(text) <= width:
            return text.ljust(width)
        return f"{text[:width - 1]}…"

    def _border(self, left: str, right: str, title: str) -> str:
        title = f" {title} "
        left_width = (self._TABLE_WIDTH - len(title)) // 2
        right_width = self._TABLE_WIDTH - len(title) - left_width
        return (left + "═" * left_width + self._paint(title, self._CYAN, bold=True) +
                "═" * right_width + right)

    def _row(self, label: str, value: str, value_color: str = "",
             value_bold: bool = False) -> str:
        value_width = self._TABLE_WIDTH - 12
        return ("║ " + self._paint(f"{label:<10}", self._CYAN, bold=True) + " " +
                self._paint(self._fit(value, value_width), value_color, bold=value_bold) + "║")

    def _print_state(self) -> None:
        if self._init_only:
            lines = [
                self._border("╔", "╗", "STARTUP ALIGNMENT"),
                *(self._row(label, value, color, value_bold=bold)
                  for label, value, color, bold in self._initialization_rows()),
                "╚" + "═" * self._TABLE_WIDTH + "╝",
            ]
            if self._redraw:
                print("\033[2J\033[H", end="")
            print("\n".join(lines), flush=True)
            return

        if self._odom is None:
            position_text = "waiting for /lio/odom"
            attitude_text = ""
            velocity_text = ""
        else:
            position = self._odom.pose.pose.position
            velocity = self._odom.twist.twist.linear
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
            frame = self._odom.header.frame_id or "unknown"
            child_frame = self._odom.child_frame_id or "unknown"
            roll, pitch, yaw = self._roll_pitch_yaw_deg(self._odom)
            position_text = (
                f"pos[{frame}] ({position.x:+.3f}, {position.y:+.3f}, {position.z:+.3f}) m")
            attitude_text = (
                f"rpy[{frame}->{child_frame}] ({roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}) deg")
            velocity_text = (
                f"vel[{frame}] ({velocity.x:+.3f}, {velocity.y:+.3f}, {velocity.z:+.3f}) m/s | "
                f"speed {speed:.3f} m/s | age {self._age(self._odom_received_at)}")

        wall_rows = self._wall_rows()
        lines = [
            self._border("╔", "╗", "WALL / LIO STATUS"),
            *(self._row(label, value, color, value_bold=bold)
              for label, value, color, bold in wall_rows),
            self._border("╠", "╣", "LIO STATE"),
            self._row("POSITION", position_text),
            self._row("ATTITUDE", attitude_text),
            self._row("VELOCITY", velocity_text),
            "╚" + "═" * self._TABLE_WIDTH + "╝",
        ]
        if self._redraw:
            print("\033[2J\033[H", end="")
        print("\n".join(lines), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=positive_rate, default=2.0,
                        help="terminal output rate in Hz (default: 2.0)")
    parser.add_argument("--wall-topic", default="/autonomy_light/front_wall_angle")
    parser.add_argument("--odom-topic", default="/lio/odom")
    parser.add_argument("--init-only", action="store_true",
                        help="show only the startup-alignment gate")
    parser.add_argument("--exit-on-init", action="store_true",
                        help="exit successfully once startup alignment is accepted")
    parser.add_argument("--color", choices=("auto", "always", "never"), default="auto",
                        help="ANSI color mode (default: auto)")
    arguments = parser.parse_args()

    if arguments.exit_on_init and not arguments.init_only:
        parser.error("--exit-on-init requires --init-only")

    rclpy.init()
    node = WallStateMonitor(arguments.wall_topic, arguments.odom_topic, arguments.rate,
                            arguments.color, arguments.init_only)
    try:
        if arguments.exit_on_init:
            # Do not call rclpy.shutdown() from a timer callback: depending on
            # the executor, spin() can remain blocked after the accepted table
            # was printed.  Owning the loop here makes INIT completion a normal,
            # deterministic return to launch.sh.
            while rclpy.ok() and not node._initialization_accepted:
                rclpy.spin_once(node, timeout_sec=0.2)
            if node._initialization_accepted:
                node._print_state()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        initialization_accepted = node._initialization_accepted
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if not arguments.exit_on_init or initialization_accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
