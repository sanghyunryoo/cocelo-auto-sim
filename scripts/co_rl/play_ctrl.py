# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Launch Isaac Sim Simulator first."""

import argparse
import math
import os
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass

from isaaclab.app import AppLauncher
import matplotlib.pyplot as plt
import numpy as np

# local imports
import cli_args  # isort: skip
from scripts.co_rl.core.runners import OffPolicyRunner
from scripts.co_rl.core.utils.str2bool import str2bool
from scripts.co_rl.core.utils.analyzer import Analyzer
from scripts.co_rl.ros2 import (
    create_auto_camera_ros_graphs,
    create_auto_stereo_camera_ros_graphs,
    create_clock_publisher,
    create_command_user_bridge,
    create_front_depth_camera_ros_graphs,
    create_front_camera_publisher,
    create_height_map_pointcloud_publisher,
    create_imu_ros_graph,
    create_mid360_pointcloud_publisher,
    create_mid360_rtx_lidar_ros_graph,
    create_robot_state_publisher,
    create_time_ros_graph,
)

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with CO-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=64, help="Number of environments to simulate.")
parser.add_argument("--algo", type=str, default="ppo", help="Name of the task.")
parser.add_argument("--stack_frames", type=int, default=None, help="Number of frames to stack.")
parser.add_argument("--plot", type=str2bool, default="False", help="Plot the data.")
parser.add_argument(
    "--analyze",
    type=str,
    nargs="+",
    default=None,
    help="Specify which data to analyze (e.g., cmd_vel joint_vel torque)."
)
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=42, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=True, help="Run in real-time, if possible.")
parser.add_argument(
    "--perf_report_interval",
    type=float,
    default=2.0,
    help="Print simulation realtime factor every N wall-clock seconds. Set <=0 to disable.",
)

parser.add_argument("--num_policy_stacks", type=int, default=2, help="Number of policy stacks.")
parser.add_argument("--num_critic_stacks", type=int, default=2, help="Number of critic stacks.")
parser.add_argument(
    "--policy_onnx_path",
    type=str,
    default="",
    help="Use an exported ONNX policy for inference instead of loading the training checkpoint policy.",
)
parser.add_argument("--hw_shoulder_kp", type=float, default=35.0, help="Shoulder joint PD stiffness Kp.")
parser.add_argument("--hw_shoulder_kd", type=float, default=0.45, help="Shoulder joint PD damping Kd.")
parser.add_argument("--hw_wheel_kp", type=float, default=0.0, help="Wheel joint PD stiffness Kp.")
parser.add_argument("--hw_wheel_kd", type=float, default=0.3, help="Wheel joint PD damping Kd.")
parser.add_argument("--action_shoulder_scale", type=float, default=0.25, help="Shoulder position action scale.")
parser.add_argument("--action_wheel_scale", type=float, default=40.0, help="Wheel velocity action scale.")
parser.add_argument("--obs_joint_pos_scale", type=float, default=1.0, help="Policy joint position observation scale.")
parser.add_argument("--obs_joint_vel_scale", type=float, default=0.15, help="Policy joint velocity observation scale.")
parser.add_argument("--obs_base_ang_vel_scale", type=float, default=0.25, help="Policy base angular velocity observation scale.")
parser.add_argument("--obs_projected_gravity_scale", type=float, default=1.0, help="Policy projected gravity observation scale.")
parser.add_argument("--obs_cmd_vel_x_scale", type=float, default=2.0, help="Policy cmd_vel linear x observation scale.")
parser.add_argument("--obs_cmd_vel_y_scale", type=float, default=0.0, help="Policy cmd_vel linear y observation scale.")
parser.add_argument("--obs_cmd_vel_yaw_scale", type=float, default=0.25, help="Policy cmd_vel angular z observation scale.")
parser.add_argument(
    "--enable_ros2_depth",
    type=str2bool,
    default=True,
    help="Publish the env_0 front depth cameras to ROS 2.",
)
parser.add_argument(
    "--ros2_depth_topic",
    type=str,
    default="/f4/front_camera/depth/image_rect_raw",
    help="ROS 2 topic for the front depth image.",
)
parser.add_argument(
    "--ros2_camera_info_topic",
    type=str,
    default="/f4/front_camera/depth/camera_info",
    help="ROS 2 topic for the front depth camera info.",
)
parser.add_argument(
    "--ros2_front_rgb_topic",
    type=str,
    default="/f4/front_camera/rgb/image_raw",
    help="ROS 2 topic for the front RGB image.",
)
parser.add_argument("--ros2_camera_rate", type=float, default=30.0, help="Camera sensor/publish rate in Hz.")
parser.add_argument("--ros2_camera_width", type=int, default=320, help="Camera sensor width in pixels.")
parser.add_argument("--ros2_camera_height", type=int, default=240, help="Camera sensor height in pixels.")
parser.add_argument(
    "--ros2_depth_frame_id",
    type=str,
    default="f4/front_camera_depth_optical_frame",
    help="Frame id used in the published ROS 2 front depth messages.",
)
parser.add_argument(
    "--enable_ros2_auto_camera",
    type=str2bool,
    default=False,
    help="Publish the env_0 A_camera_link auto depth/RGB streams to ROS 2.",
)
parser.add_argument(
    "--ros2_auto_depth_topic",
    type=str,
    default="/f4/adas_camera/depth/image_rect_raw",
    help="ROS 2 topic for the A_camera_link auto depth image.",
)
parser.add_argument(
    "--ros2_auto_depth_camera_info_topic",
    type=str,
    default="/f4/adas_camera/depth/camera_info",
    help="ROS 2 topic for the A_camera_link auto depth camera info.",
)
parser.add_argument(
    "--ros2_auto_depth_frame_id",
    type=str,
    default="A_camera_link",
    help="Frame id used in the published A_camera_link auto depth messages.",
)
parser.add_argument(
    "--ros2_auto_rgb_topic",
    type=str,
    default="/f4/adas_camera/rgb/image_raw",
    help="ROS 2 topic for the A_camera_link auto RGB image.",
)
parser.add_argument(
    "--ros2_auto_rgb_camera_info_topic",
    type=str,
    default="/f4/adas_camera/rgb/camera_info",
    help="ROS 2 topic for the A_camera_link auto RGB camera info.",
)
parser.add_argument(
    "--ros2_auto_rgb_frame_id",
    type=str,
    default="A_camera_link",
    help="Frame id used in the published A_camera_link auto RGB messages.",
)
parser.add_argument(
    "--enable_ros2_stereo",
    type=str2bool,
    default=False,
    help="Publish the env_0 A_camera_link virtual D435i left/right stereo images to ROS 2.",
)
parser.add_argument(
    "--ros2_auto_left_image_topic",
    type=str,
    default="/f4/adas_camera/left/image_raw",
    help="ROS 2 topic for the A_camera_link virtual D435i left image.",
)
parser.add_argument(
    "--ros2_auto_right_image_topic",
    type=str,
    default="/f4/adas_camera/right/image_raw",
    help="ROS 2 topic for the A_camera_link virtual D435i right image.",
)
parser.add_argument(
    "--ros2_auto_left_camera_info_topic",
    type=str,
    default="/f4/adas_camera/left/camera_info",
    help="ROS 2 topic for the A_camera_link virtual D435i left camera info.",
)
parser.add_argument(
    "--ros2_auto_right_camera_info_topic",
    type=str,
    default="/f4/adas_camera/right/camera_info",
    help="ROS 2 topic for the A_camera_link virtual D435i right camera info.",
)
parser.add_argument(
    "--ros2_auto_left_frame_id",
    type=str,
    default="A_camera_link_left_optical",
    help="Frame id used in the published A_camera_link virtual D435i left messages.",
)
parser.add_argument(
    "--ros2_auto_right_frame_id",
    type=str,
    default="A_camera_link_right_optical",
    help="Frame id used in the published A_camera_link virtual D435i right messages.",
)
parser.add_argument(
    "--enable_ros2_imu",
    type=str2bool,
    default=True,
    help="Publish the env_0 IMU data to ROS 2.",
)
parser.add_argument(
    "--ros2_imu_topic",
    type=str,
    default="/f4/imu",
    help="ROS 2 topic for the IMU message.",
)
parser.add_argument(
    "--ros2_imu_frame_id",
    type=str,
    default="f4/base_link",
    help="Frame id used in the published ROS 2 IMU messages.",
)
parser.add_argument("--ros2_imu_rate", type=float, default=200.0, help="Base IMU publish rate in Hz.")
parser.add_argument(
    "--enable_ros2_front_camera_imu",
    type=str2bool,
    default=True,
    help="Publish the env_0 front camera IMU data to ROS 2.",
)
parser.add_argument(
    "--ros2_front_camera_imu_topic",
    type=str,
    default="/f4/front_camera/imu",
    help="ROS 2 topic for the front camera IMU message.",
)
parser.add_argument(
    "--ros2_front_camera_imu_frame_id",
    type=str,
    default="f4/front_camera_link",
    help="Frame id used in the published front camera IMU messages.",
)
parser.add_argument(
    "--ros2_time_topic",
    type=str,
    default="time",
    help="ROS 2 topic for the simulation time as std_msgs/Float64.",
)
parser.add_argument(
    "--enable_ros2_robot_state",
    type=str2bool,
    default=False,
    help="Publish env_0 joint states for RViz. Keep disabled when sensor TFs must not be produced downstream.",
)
parser.add_argument(
    "--ros2_joint_states_topic",
    type=str,
    default="/f4/joint_states",
    help="ROS 2 topic for the JointState message.",
)
parser.add_argument(
    "--ros2_world_frame_id",
    type=str,
    default="world",
    help="World frame id used for root TF and path_gt.",
)
parser.add_argument(
    "--ros2_base_frame_id",
    type=str,
    default="f4/base_link",
    help="Robot base frame id used as the child frame for world->base TF.",
)
parser.add_argument(
    "--ros2_path_gt_topic",
    type=str,
    default="/path_gt",
    help="ROS 2 topic for the lidar-referenced nav_msgs/Path ground truth.",
)
parser.add_argument(
    "--ros2_gt_odom_topic",
    type=str,
    default="/gt/lidar_odom",
    help="ROS 2 topic for lidar-referenced ground-truth Odometry.",
)
parser.add_argument(
    "--ros2_gt_child_frame_id",
    type=str,
    default="f4/lidar_link",
    help="Child frame identified by path_gt and ground-truth Odometry poses.",
)
parser.add_argument(
    "--ros2_slam_diagnostics_csv",
    type=str,
    default="",
    help="Optional CSV path recording synchronized lidar GT and LIO odometry.",
)
parser.add_argument(
    "--ros2_path_gt_max_poses",
    type=int,
    default=5000,
    help="Maximum number of poses retained in the published path_gt message.",
)
parser.add_argument(
    "--ros2_publish_root_tf",
    type=str2bool,
    default=True,
    help="Publish the root-frame to robot-base TF. Disable when SLAM owns the map/odom/base chain.",
)
parser.add_argument(
    "--ros2_path_gt_relative_to_initial",
    type=str2bool,
    default=False,
    help="Express path_gt relative to the initial simulated robot pose.",
)
parser.add_argument(
    "--enable_ros2_height_map",
    type=str2bool,
    default=True,
    help="Publish the env_0 IsaacLab height scanner ray hits as a ROS 2 PointCloud2.",
)
parser.add_argument(
    "--ros2_height_map_topic",
    type=str,
    default="/f4/height_map/points",
    help="ROS 2 topic for the height map PointCloud2 message.",
)
parser.add_argument(
    "--ros2_height_map_frame_id",
    type=str,
    default="world",
    help="Frame id used in the published height map PointCloud2 messages.",
)
parser.add_argument(
    "--ros2_height_map_sensor",
    type=str,
    default="height_scanner_fov",
    help="IsaacLab scene sensor name used as the height map source.",
)
parser.add_argument(
    "--enable_ros2_mid360",
    type=str2bool,
    default=True,
    help="Publish the env_0 lidar as a ROS 2 PointCloud2.",
)
parser.add_argument(
    "--enable_ros2_mid360_rtx",
    type=str2bool,
    default=False,
    help=(
        "Use Isaac RTX LiDAR for point cloud publishing instead of the RayCaster fallback. "
        "The RayCaster fallback is used by default because it works without extra sensor assets."
    ),
)
parser.add_argument(
    "--ros2_mid360_lidar_topic",
    type=str,
    default="/f4/lidar/points",
    help="ROS 2 topic for the lidar PointCloud2 message.",
)
parser.add_argument(
    "--ros2_mid360_frame_id",
    type=str,
    default="f4/lidar_link",
    help="Frame id used in the lidar and lidar IMU messages.",
)
parser.add_argument(
    "--ros2_mid360_sensor",
    type=str,
    default="lidar",
    help="IsaacLab scene sensor name used as the RayCaster lidar source.",
)
parser.add_argument(
    "--enable_ros2_mid360_imu",
    type=str2bool,
    default=True,
    help="Publish the lidar IMU to ROS 2.",
)
parser.add_argument(
    "--ros2_mid360_imu_topic",
    type=str,
    default="/f4/lidar/imu",
    help="ROS 2 topic for the lidar IMU message.",
)
parser.add_argument(
    "--ros2_mid360_imu_sensor",
    type=str,
    default="lidar_imu",
    help="IsaacLab scene sensor name used as the lidar IMU source.",
)
parser.add_argument("--ros2_mid360_rate", type=float, default=10.0, help="Lidar publish rate in Hz.")

# teleop options
parser.add_argument("--teleop", type=str2bool, default=True, help="Use keyboard teleoperation.")
parser.add_argument(
    "--teleop_use_stdin",
    type=str2bool,
    default=False,
    help="Use terminal stdin for teleop instead of pynput global keyboard hooks.",
)
parser.add_argument(
    "--teleop_delta",
    type=float,
    default=2.0,
    help="Command change rate per second while key is held.",
)
parser.add_argument(
    "--enable_ros2_command_user",
    type=str2bool,
    default=True,
    help="Route teleop/autonomy commands through core/msg/CommandUser over ROS 2.",
)
parser.add_argument(
    "--ros2_command_user_topic",
    type=str,
    default="/control_command/user_odom",
    help="ROS 2 topic for core/msg/CommandUser commands.",
)
parser.add_argument(
    "--ros2_command_user_publish_keyboard",
    type=str2bool,
    default=True,
    help="Publish local keyboard teleop commands as core/msg/CommandUser.",
)
parser.add_argument(
    "--ros2_command_user_rate",
    type=float,
    default=10.0,
    help="Publish local keyboard CommandUser messages at this rate in Hz.",
)
parser.add_argument(
    "--ros2_nav_cmd_vel_topic",
    type=str,
    default="/nav2/cmd_vel",
    help="geometry_msgs/Twist topic used by Nav2 to command the simulated robot.",
)

# append CO-RL cli arguments
cli_args.add_co_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# always enable cameras to record video / ROS 2 camera streams
if args_cli.video or args_cli.enable_ros2_depth or args_cli.enable_ros2_auto_camera or args_cli.enable_ros2_stereo:
    args_cli.enable_cameras = True

# teleop은 보통 단일 환경이 맞음
if args_cli.teleop and args_cli.num_envs != 1:
    print(f"[INFO] --teleop enabled, overriding num_envs {args_cli.num_envs} -> 1")
    args_cli.num_envs = 1

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch

from scripts.co_rl.core.runners import OnPolicyRunner, SRMOnPolicyRunner
from isaaclab.utils.dict import print_dict

from scripts.co_rl.core.wrapper import (
    CoRlPolicyRunnerCfg,
    CoRlVecEnvWrapper,
    export_env_as_pdf,
    export_policy_as_jit,
    export_policy_as_onnx,
    export_srm_as_onnx,
)

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
try:
    # Isaac Lab 2.3 and newer moved this helper into isaaclab_rl.
    from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint
except ModuleNotFoundError:
    from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

# Import extensions to set up environment tasks
import lab.flamingo.tasks  # noqa: F401
try:
    from lab.flamingo.isaaclab.isaaclab.envs import ManagerBasedConstraintRLEnv, ManagerBasedConstraintRLEnvCfg
except ModuleNotFoundError:
    ManagerBasedConstraintRLEnv = ()
    ManagerBasedConstraintRLEnvCfg = ()

from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg


@dataclass
class AxisBinding:
    axis_name: str
    pos_key: str
    neg_key: str
    min_value: float
    max_value: float


@dataclass
class TeleopCommandSpec:
    obs_slice: slice
    command_dim: int
    obs_scale: torch.Tensor
    obs_clip_min: torch.Tensor | None
    obs_clip_max: torch.Tensor | None
    cmd_min: torch.Tensor
    cmd_max: torch.Tensor
    axis_names: list[str]
    bindings: list[AxisBinding]
    command_term_name: str


def _unwrap_env(env):
    cur = env
    seen = set()
    while hasattr(cur, "unwrapped") and id(cur) not in seen:
        seen.add(id(cur))
        nxt = cur.unwrapped
        if nxt is cur:
            break
        cur = nxt
    return cur


def _safe_get_attr(obj, name, default=None):
    return getattr(obj, name, default) if obj is not None else default


def _hide_isaac_debug_visuals():
    """Hide IsaacLab debug marker prims so they do not appear in rendered camera images."""
    try:
        import omni.usd
        from pxr import UsdGeom
    except Exception as exc:
        print(f"[WARN] Could not hide Isaac debug visual prims: {exc}")
        return

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return

    hidden_paths = []
    for path in ("/Visuals", "/Debug"):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()
            hidden_paths.append(path)

    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        lowered = path.lower()
        if "/visuals/" in lowered or "raycaster" in lowered or "pointcloud" in lowered:
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()
                hidden_paths.append(path)

    if hidden_paths:
        print(f"[INFO] Hid {len(set(hidden_paths))} Isaac debug visual prims from rendering.")


def _sanitize_reflective_surface_materials():
    """Reduce reflective material fireflies in the loaded USD scene without editing source assets."""
    if os.environ.get("SANITIZE_SCENE_MATERIALS", "1").lower() in ("0", "false", "no", "off"):
        return

    try:
        import omni.usd
        from pxr import Sdf, UsdShade
    except Exception as exc:
        print(f"[WARN] Could not sanitize scene materials: {exc}")
        return

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return

    float_inputs = {
        "roughness": 1.0,
        "reflection_roughness": 1.0,
        "specular_roughness": 1.0,
        "metallic": 0.0,
        "metalness": 0.0,
        "specular": 0.0,
        "specular_level": 0.0,
        "clearcoat": 0.0,
        "coat_weight": 0.0,
    }
    bool_inputs = {
        "enable_emission": False,
        "emission_enabled": False,
    }

    edited = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        existing_inputs = {shader_input.GetBaseName(): shader_input for shader_input in shader.GetInputs()}

        for name, value in float_inputs.items():
            shader_input = existing_inputs.get(name)
            if shader_input is not None:
                shader_input.Set(float(value))
                edited += 1
            elif name in ("roughness", "metallic"):
                shader.CreateInput(name, Sdf.ValueTypeNames.Float).Set(float(value))
                edited += 1

        for name, value in bool_inputs.items():
            shader_input = existing_inputs.get(name)
            if shader_input is not None:
                shader_input.Set(bool(value))
                edited += 1

    if edited:
        print(f"[INFO] Sanitized {edited} scene material inputs to reduce white sparkle/fireflies.")


def _expand_to_dim(value, dim, device):
    if value is None:
        return None
    if isinstance(value, torch.Tensor):
        t = value.to(device=device, dtype=torch.float32).flatten()
    elif isinstance(value, (list, tuple)):
        t = torch.tensor(value, dtype=torch.float32, device=device).flatten()
    else:
        t = torch.tensor([float(value)], dtype=torch.float32, device=device)

    if t.numel() == 1:
        t = t.repeat(dim)
    elif t.numel() != dim:
        t = torch.full((dim,), float(torch.mean(t).item()), dtype=torch.float32, device=device)
    return t


def _prod_tuple(shape):
    if isinstance(shape, int):
        return shape
    out = 1
    for v in shape:
        out *= v
    return out


def _get_env_dt(env_unwrapped) -> float:
    step_dt = _safe_get_attr(env_unwrapped, "step_dt", None)
    if step_dt is not None:
        try:
            return float(step_dt)
        except Exception:
            pass

    sim_cfg = _safe_get_attr(_safe_get_attr(env_unwrapped, "cfg", None), "sim", None)
    sim_dt = _safe_get_attr(sim_cfg, "dt", None)
    if sim_dt is not None:
        try:
            return float(sim_dt)
        except Exception:
            pass

    return 1.0 / 60.0


def _choose_default_binding(axis_name: str, used_pairs: set[tuple[str, str]]) -> tuple[str, str]:
    name = axis_name.lower()

    preferred = []
    if "lin_vel_x" in name or name in ("x", "vx", "vel_x"):
        preferred.append(("w", "s"))
    if "lin_vel_y" in name or name in ("y", "vy", "vel_y"):
        preferred.append(("q", "e"))
    if "ang_vel_z" in name or "yaw" in name or name in ("wz", "omega_z", "yaw_rate"):
        preferred.append(("a", "d"))
    if "pos_z" in name or name in ("z", "height", "base_height"):
        preferred.append(("z", "x"))
    if "heading" in name:
        preferred.append(("z", "x"))

    fallback_pairs = [
        ("z", "x"),
        ("c", "v"),
        ("t", "g"),
        ("u", "j"),
        ("i", "k"),
        ("o", "l"),
    ]

    for pair in preferred + fallback_pairs:
        if pair not in used_pairs:
            used_pairs.add(pair)
            return pair

    # 정말 드문 경우
    pair = ("n", "m")
    used_pairs.add(pair)
    return pair


def _read_command_ranges_from_cfg(env_unwrapped, command_dim: int, device):
    """cfg.commands.*.ranges에서 command 이름/축 이름/범위를 best-effort로 읽는다."""
    cfg = _safe_get_attr(env_unwrapped, "cfg", None)
    commands_cfg = _safe_get_attr(cfg, "commands", None)

    if commands_cfg is None:
        return None, None, None, None

    candidate_terms = []
    for name in dir(commands_cfg):
        if name.startswith("_"):
            continue
        term_cfg = getattr(commands_cfg, name)
        ranges = _safe_get_attr(term_cfg, "ranges", None)
        if ranges is not None:
            candidate_terms.append((name, term_cfg, ranges))

    if len(candidate_terms) == 0:
        return None, None, None, None

    preferred = None
    for name, term_cfg, ranges in candidate_terms:
        lname = name.lower()
        if any(k in lname for k in ["base_velocity", "velocity", "command", "cmd"]):
            preferred = (name, term_cfg, ranges)
            break
    if preferred is None:
        preferred = candidate_terms[0]

    term_name, _, ranges = preferred

    mins = []
    maxs = []
    axis_names = []

    ordered_keys = [
        "lin_vel_x",
        "lin_vel_y",
        "ang_vel_z",
        "pos_z",
        "heading",
    ]

    for key in ordered_keys:
        val = _safe_get_attr(ranges, key, None)
        if val is not None and isinstance(val, (tuple, list)) and len(val) == 2:
            mins.append(float(val[0]))
            maxs.append(float(val[1]))
            axis_names.append(key)

    if len(mins) < command_dim:
        for name in dir(ranges):
            if name.startswith("_") or name in ordered_keys:
                continue
            val = getattr(ranges, name)
            if isinstance(val, (tuple, list)) and len(val) == 2:
                try:
                    lo = float(val[0])
                    hi = float(val[1])
                    mins.append(lo)
                    maxs.append(hi)
                    axis_names.append(name)
                    if len(mins) == command_dim:
                        break
                except Exception:
                    pass

    if len(mins) == 0:
        return None, None, None, None

    if len(mins) < command_dim:
        last_min = mins[-1]
        last_max = maxs[-1]
        while len(mins) < command_dim:
            mins.append(last_min)
            maxs.append(last_max)
            axis_names.append(f"extra_{len(axis_names)}")

    mins = mins[:command_dim]
    maxs = maxs[:command_dim]
    axis_names = axis_names[:command_dim]

    cmd_min = torch.tensor(mins, dtype=torch.float32, device=device)
    cmd_max = torch.tensor(maxs, dtype=torch.float32, device=device)
    return cmd_min, cmd_max, axis_names, term_name


def _default_axis_names_from_dim(dim: int) -> list[str]:
    if dim == 1:
        return ["lin_vel_x"]
    if dim == 2:
        return ["lin_vel_x", "ang_vel_z"]
    if dim == 3:
        return ["lin_vel_x", "lin_vel_y", "ang_vel_z"]
    if dim >= 4:
        return ["lin_vel_x", "lin_vel_y", "ang_vel_z", "pos_z"][:dim]
    return [f"axis_{i}" for i in range(dim)]


def _build_bindings(axis_names: list[str], cmd_min: torch.Tensor, cmd_max: torch.Tensor) -> list[AxisBinding]:
    used_pairs: set[tuple[str, str]] = set()
    bindings = []
    for i, axis_name in enumerate(axis_names):
        pos_key, neg_key = _choose_default_binding(axis_name, used_pairs)
        bindings.append(
            AxisBinding(
                axis_name=axis_name,
                pos_key=pos_key,
                neg_key=neg_key,
                min_value=float(cmd_min[i].item()),
                max_value=float(cmd_max[i].item()),
            )
        )
    return bindings


def _apply_wasd_drive_bindings(bindings: list[AxisBinding]) -> list[AxisBinding]:
    """Force a familiar W/S + A/D layout for mobile-base style commands."""
    axis_to_pair = {
        "lin_vel_x": ("w", "s"),
        "ang_vel_z": ("a", "d"),
        "lin_vel_y": ("q", "e"),
        "pos_z": ("z", "x"),
        "heading": ("a", "d"),
    }

    updated: list[AxisBinding] = []
    used_pairs: set[tuple[str, str]] = set()

    for binding in bindings:
        pair = axis_to_pair.get(binding.axis_name.lower())
        if pair is None or pair in used_pairs:
            updated.append(binding)
            continue
        used_pairs.add(pair)
        updated.append(
            AxisBinding(
                axis_name=binding.axis_name,
                pos_key=pair[0],
                neg_key=pair[1],
                min_value=binding.min_value,
                max_value=binding.max_value,
            )
        )

    return updated


def _expand_command_scale(scale, dim: int, device) -> torch.Tensor:
    if scale is None:
        return torch.ones(dim, dtype=torch.float32, device=device)

    if isinstance(scale, torch.Tensor):
        out = scale.to(device=device, dtype=torch.float32).flatten()
    elif isinstance(scale, (list, tuple)):
        out = torch.tensor(scale, dtype=torch.float32, device=device).flatten()
    else:
        out = torch.tensor([float(scale)], dtype=torch.float32, device=device)

    if out.numel() < dim:
        pad = torch.ones(dim - out.numel(), dtype=torch.float32, device=device)
        out = torch.cat((out, pad), dim=0)
    elif out.numel() > dim:
        out = out[:dim]
    return out


def _find_command_obs_term(env_unwrapped, command_term_name: str):
    obs_mgr = _safe_get_attr(env_unwrapped, "observation_manager", None)
    obs_cfg = _safe_get_attr(_safe_get_attr(env_unwrapped, "cfg", None), "observations", None)
    policy_cfg = _safe_get_attr(obs_cfg, "none_stack_policy", None)

    if obs_mgr is None or policy_cfg is None:
        return None, None, None

    active_terms = _safe_get_attr(obs_mgr, "active_terms", {}).get("none_stack_policy", [])
    term_dims = _safe_get_attr(obs_mgr, "group_obs_term_dim", {}).get("none_stack_policy", [])

    offset = 0
    for term_name, term_dim in zip(active_terms, term_dims):
        flat_dim = _prod_tuple(term_dim)
        term_cfg = _safe_get_attr(policy_cfg, term_name, None)
        params = _safe_get_attr(term_cfg, "params", None)
        lower_name = term_name.lower()

        matched = False
        if isinstance(params, dict) and params.get("command_name") == command_term_name:
            matched = True
        elif "velocity_command" in lower_name or "velocity_commands" in lower_name or "base_velocity" in lower_name:
            matched = True

        if matched:
            return term_name, slice(offset, offset + flat_dim), term_cfg
        offset += flat_dim

    return None, None, None


def _set_env_command(env_unwrapped, command_term_name: str, command: torch.Tensor) -> bool:
    command_manager = _safe_get_attr(env_unwrapped, "command_manager", None)
    if command_manager is None:
        return False

    term = None
    get_term = _safe_get_attr(command_manager, "get_term", None)
    if callable(get_term):
        try:
            term = get_term(command_term_name)
        except Exception:
            term = None

    if term is not None:
        for attr_name in ("command", "vel_command_b", "command_b"):
            tensor = _safe_get_attr(term, attr_name, None)
            if isinstance(tensor, torch.Tensor) and tensor.shape[0] == command.shape[0] and tensor.shape[1] >= command.shape[1]:
                tensor[:, : command.shape[1]].copy_(command)
                return True

    get_command = _safe_get_attr(command_manager, "get_command", None)
    if callable(get_command):
        try:
            tensor = get_command(command_term_name)
        except Exception:
            tensor = None
        if isinstance(tensor, torch.Tensor) and tensor.shape[0] == command.shape[0] and tensor.shape[1] >= command.shape[1]:
            tensor[:, : command.shape[1]].copy_(command)
            return True

    return False


def _command_tensor_from_array(command, spec: TeleopCommandSpec, num_envs: int, device: torch.device) -> torch.Tensor:
    command_tensor = torch.as_tensor(command, dtype=torch.float32, device=device).flatten()
    if command_tensor.numel() < spec.command_dim:
        padded = torch.zeros(spec.command_dim, dtype=torch.float32, device=device)
        padded[: command_tensor.numel()] = command_tensor
        command_tensor = padded
    command_tensor = command_tensor[: spec.command_dim].unsqueeze(0).repeat(num_envs, 1)
    return torch.max(
        torch.min(command_tensor, spec.cmd_max.unsqueeze(0)),
        spec.cmd_min.unsqueeze(0),
    )


def _apply_command_to_obs(obs: torch.Tensor, spec: TeleopCommandSpec, command: torch.Tensor) -> torch.Tensor:
    obs_mod = obs.clone()
    scaled_command = command * spec.obs_scale.unsqueeze(0)

    if spec.obs_clip_min is not None and spec.obs_clip_max is not None:
        scaled_command = torch.max(
            torch.min(scaled_command, spec.obs_clip_max.unsqueeze(0)),
            spec.obs_clip_min.unsqueeze(0),
        )

    obs_mod[:, spec.obs_slice] = scaled_command
    return obs_mod


def infer_teleop_command_spec(env, obs_tensor: torch.Tensor) -> TeleopCommandSpec:
    env_unwrapped = _unwrap_env(env)
    device = obs_tensor.device

    cmd_min, cmd_max, axis_names, cfg_command_term_name = _read_command_ranges_from_cfg(
        env_unwrapped, 4, device
    )
    command_term_name = cfg_command_term_name if cfg_command_term_name is not None else "base_velocity"

    selected_name, nonstack_slice, term_cfg = _find_command_obs_term(env_unwrapped, command_term_name)

    selected_dim = None if nonstack_slice is None else nonstack_slice.stop - nonstack_slice.start
    selected_slice = None

    if selected_dim is None:
        total_dim = int(obs_tensor.shape[-1])
        if total_dim >= 4:
            selected_dim = 4
            selected_start = total_dim - 4
            selected_name = "fallback_last4"
        elif total_dim >= 3:
            selected_dim = 3
            selected_start = total_dim - 3
            selected_name = "fallback_last3"
        elif total_dim >= 2:
            selected_dim = 2
            selected_start = total_dim - 2
            selected_name = "fallback_last2"
        elif total_dim >= 1:
            selected_dim = 1
            selected_start = total_dim - 1
            selected_name = "fallback_last1"
        else:
            raise RuntimeError(f"Observation dim is {total_dim}; cannot infer command block.")

        selected_slice = slice(selected_start, selected_start + selected_dim)
    else:
        stack_dim = _safe_get_attr(_safe_get_attr(env, "policy_state_handler", None), "stack_dim", 0)
        total_frames = _safe_get_attr(_safe_get_attr(env, "policy_state_handler", None), "total_frames", 1)
        nonstack_offset = int(stack_dim) * int(total_frames)
        selected_slice = slice(nonstack_offset + nonstack_slice.start, nonstack_offset + nonstack_slice.stop)

    term_params = _safe_get_attr(term_cfg, "params", None)
    raw_obs_scale = term_params.get("scale") if isinstance(term_params, dict) else None
    if raw_obs_scale is None:
        raw_obs_scale = _safe_get_attr(term_cfg, "scale", 1.0)
    obs_scale = _expand_command_scale(raw_obs_scale, selected_dim, device)
    clip_cfg = _safe_get_attr(term_cfg, "clip", None)

    obs_clip_min = None
    obs_clip_max = None
    if isinstance(clip_cfg, (tuple, list)) and len(clip_cfg) == 2:
        obs_clip_min = _expand_to_dim(clip_cfg[0], selected_dim, device)
        obs_clip_max = _expand_to_dim(clip_cfg[1], selected_dim, device)

    if cmd_min is None or cmd_max is None:
        if obs_clip_min is not None and obs_clip_max is not None:
            cmd_min = obs_clip_min.clone()
            cmd_max = obs_clip_max.clone()
        else:
            cmd_min = torch.full((selected_dim,), -1.0, dtype=torch.float32, device=device)
            cmd_max = torch.full((selected_dim,), 1.0, dtype=torch.float32, device=device)
    else:
        cmd_min = cmd_min[:selected_dim]
        cmd_max = cmd_max[:selected_dim]

    if axis_names is None:
        axis_names = _default_axis_names_from_dim(selected_dim)
    else:
        axis_names = axis_names[:selected_dim]

    bindings = _build_bindings(axis_names, cmd_min, cmd_max)
    bindings = _apply_wasd_drive_bindings(bindings)

    print("[TELEOP] --------------------------------------------------")
    print(f"[TELEOP] command obs term : {selected_name}")
    print(f"[TELEOP] command cfg term : {command_term_name}")
    print(f"[TELEOP] command slice    : {selected_slice.start}:{selected_slice.stop}")
    print(f"[TELEOP] command dim      : {selected_dim}")
    print(f"[TELEOP] obs scale        : {obs_scale.detach().cpu().numpy()}")
    print("[TELEOP] axis mappings:")
    for i, b in enumerate(bindings):
        print(
            f"[TELEOP]   axis {i}: {b.axis_name:>12s} | "
            f"{b.neg_key.upper()}/{b.pos_key.upper()} | "
            f"range [{b.min_value:.4f}, {b.max_value:.4f}]"
        )
    print("[TELEOP] drive keys: W/S=forward/backward, A/D=turn left/right")
    print("[TELEOP] extra keys: Q/E=lateral, Z/X=height, SPACE=reset, R=print current command")
    print("[TELEOP] --------------------------------------------------")

    return TeleopCommandSpec(
        obs_slice=selected_slice,
        command_dim=selected_dim,
        obs_scale=obs_scale,
        obs_clip_min=obs_clip_min,
        obs_clip_max=obs_clip_max,
        cmd_min=cmd_min,
        cmd_max=cmd_max,
        axis_names=axis_names,
        bindings=bindings,
        command_term_name=command_term_name,
    )


class KeyboardTeleop:
    """키를 누를 때만 command 생성. 키를 떼면 즉시 0."""

    def __init__(
        self,
        num_envs: int,
        device: torch.device,
        spec: TeleopCommandSpec,
        delta: float = 2.0,
        dt: float = 1.0 / 60.0,
        use_stdin: bool = False,
    ):
        self.num_envs = num_envs
        self.device = device
        self.spec = spec
        self.delta = float(delta)
        self.dt = float(dt)

        self.current_command = torch.zeros((num_envs, spec.command_dim), dtype=torch.float32, device=device)

        self.pressed = set()
        self.lock = threading.Lock()
        self.listener = None
        self._stdin_fd = None
        self._stdin_settings = None
        self._use_stdin = False
        self._force_stdin = bool(use_stdin)
        self._start_listener()

    def _start_listener(self):
        if self._force_stdin:
            self._start_stdin_listener("[TELEOP] using terminal stdin keyboard control.")
            return

        try:
            from pynput import keyboard
        except ImportError:
            self._start_stdin_listener("[WARN] pynput is not installed; using terminal keyboard teleop fallback.")
            return

        special_key_map = {
            keyboard.Key.up: "w",
            keyboard.Key.down: "s",
            keyboard.Key.left: "a",
            keyboard.Key.right: "d",
            keyboard.Key.space: "space",
        }

        def on_press(key):
            with self.lock:
                try:
                    if key.char is not None:
                        self.pressed.add(key.char.lower())
                except AttributeError:
                    mapped_key = special_key_map.get(key)
                    if mapped_key is not None:
                        self.pressed.add(mapped_key)

        def on_release(key):
            with self.lock:
                try:
                    if key.char is not None:
                        self.pressed.discard(key.char.lower())
                except AttributeError:
                    mapped_key = special_key_map.get(key)
                    if mapped_key is not None:
                        self.pressed.discard(mapped_key)

        self.listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        self.listener.daemon = True
        self.listener.start()

    def _start_stdin_listener(self, message: str):
        if sys.stdin.isatty():
            self._stdin_fd = sys.stdin.fileno()
            self._stdin_settings = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
            self._use_stdin = True
            print(message)
        else:
            print("[WARN] stdin is not a TTY; keyboard teleop uses zero command.")

    def _poll_stdin(self):
        if not self._use_stdin:
            return
        keys = set()
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                break
            char = sys.stdin.read(1)
            if not char:
                break
            if char == "\x1b":
                seq = ""
                for _ in range(2):
                    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if not readable:
                        break
                    seq += sys.stdin.read(1)
                arrow_key_map = {"[A": "w", "[B": "s", "[C": "d", "[D": "a"}
                mapped_key = arrow_key_map.get(seq)
                if mapped_key is not None:
                    keys.add(mapped_key)
            elif char == " ":
                keys.add("space")
            else:
                keys.add(char.lower())
        with self.lock:
            self.pressed = keys

    def _ramp_axis(self, pos_key: str, neg_key: str, axis: int):
        if axis >= self.spec.command_dim:
            return

        lo = float(self.spec.cmd_min[axis].item())
        hi = float(self.spec.cmd_max[axis].item())
        step = self.delta * self.dt

        pos_pressed = pos_key in self.pressed
        neg_pressed = neg_key in self.pressed

        if pos_pressed == neg_pressed:
            self.current_command[:, axis] = 0.0
            return

        if pos_pressed:
            self.current_command[:, axis] += step
            self.current_command[:, axis] = torch.clamp(self.current_command[:, axis], 0.0, hi)
        else:
            self.current_command[:, axis] -= step
            self.current_command[:, axis] = torch.clamp(self.current_command[:, axis], lo, 0.0)

    def update(self):
        self._poll_stdin()
        with self.lock:
            self.pressed = set(self.pressed)

        if "space" in self.pressed:
            self.current_command.zero_()
        else:
            for axis_idx, binding in enumerate(self.spec.bindings):
                self._ramp_axis(binding.pos_key, binding.neg_key, axis_idx)

        self.current_command = torch.max(
            torch.min(self.current_command, self.spec.cmd_max.unsqueeze(0)),
            self.spec.cmd_min.unsqueeze(0),
        )

        if "r" in self.pressed:
            print(f"[TELEOP] command: {self.current_command[0].detach().cpu().numpy()}")

        return self.current_command.clone()

    def apply_to_env(self, env) -> bool:
        env_unwrapped = _unwrap_env(env)
        return _set_env_command(env_unwrapped, self.spec.command_term_name, self.current_command)

    def set_command(self, command) -> None:
        command_tensor = torch.as_tensor(command, dtype=torch.float32, device=self.device).flatten()
        if command_tensor.numel() < self.spec.command_dim:
            padded = torch.zeros(self.spec.command_dim, dtype=torch.float32, device=self.device)
            padded[: command_tensor.numel()] = command_tensor
            command_tensor = padded
        command_tensor = command_tensor[: self.spec.command_dim].unsqueeze(0).repeat(self.num_envs, 1)
        self.current_command = torch.max(
            torch.min(command_tensor, self.spec.cmd_max.unsqueeze(0)),
            self.spec.cmd_min.unsqueeze(0),
        )

    def apply_to_obs(self, obs: torch.Tensor) -> torch.Tensor:
        return _apply_command_to_obs(obs, self.spec, self.current_command)

    def stop(self):
        if self.listener is not None:
            self.listener.stop()
            self.listener = None
        if self._stdin_fd is not None and self._stdin_settings is not None:
            try:
                termios.tcsetattr(self._stdin_fd, termios.TCSAFLUSH, self._stdin_settings)
            except termios.error:
                termios.tcsetattr(self._stdin_fd, termios.TCSANOW, self._stdin_settings)
            self._stdin_fd = None
            self._stdin_settings = None
            self._use_stdin = False


class OnnxPolicy:
    def __init__(self, path: str, device: torch.device):
        import onnxruntime as ort

        if not os.path.isfile(path):
            raise FileNotFoundError(f"ONNX policy not found: {path}")

        providers = ["CPUExecutionProvider"]
        available = set(ort.get_available_providers())
        if "CUDAExecutionProvider" in available and "cuda" in str(device):
            providers.insert(0, "CUDAExecutionProvider")

        self.path = path
        self.device = device
        self.session = ort.InferenceSession(path, providers=providers)
        self.inputs = [item.name for item in self.session.get_inputs()]
        self.outputs = [item.name for item in self.session.get_outputs()]
        self.state = {}

        for item in self.session.get_inputs():
            shape = [1 if not isinstance(dim, int) or dim <= 0 else dim for dim in item.shape]
            if item.name in ("h_in", "c_in"):
                self.state[item.name] = np.zeros(shape, dtype=np.float32)

        print(
            f"[INFO] Using ONNX policy: {path} "
            f"inputs={self.inputs} outputs={self.outputs} providers={self.session.get_providers()}"
        )

    def __call__(self, obs: torch.Tensor) -> torch.Tensor:
        obs_np = obs.detach().cpu().numpy().astype(np.float32, copy=False)
        feed = {}
        for name in self.inputs:
            if name == "obs":
                feed[name] = obs_np
            elif name in self.state:
                feed[name] = self.state[name]
            else:
                raise RuntimeError(f"Unsupported ONNX policy input '{name}' in {self.path}")

        outputs = self.session.run(None, feed)
        output_map = dict(zip(self.outputs, outputs))
        if "h_out" in output_map:
            self.state["h_in"] = output_map["h_out"]
        if "c_out" in output_map:
            self.state["c_in"] = output_map["c_out"]

        actions = output_map.get("actions", outputs[0])
        return torch.as_tensor(actions, dtype=torch.float32, device=self.device)


def _disable_terminations_for_teleop(env_cfg):
    env_cfg.episode_length_s = 1.0e9

    terminations_cfg = _safe_get_attr(env_cfg, "terminations", None)
    if terminations_cfg is None:
        return

    disabled_terms = []
    for name in dir(terminations_cfg):
        if name.startswith("_"):
            continue
        term = getattr(terminations_cfg, name)
        if callable(term):
            continue
        setattr(terminations_cfg, name, None)
        disabled_terms.append(name)

    if disabled_terms:
        print(f"[TELEOP] disabled terminations: {', '.join(disabled_terms)}")
    print(f"[TELEOP] episode_length_s set to {env_cfg.episode_length_s}")


def _configure_runtime_sensors(env_cfg, args):
    scene_cfg = _safe_get_attr(env_cfg, "scene", None)
    if scene_cfg is None:
        return

    camera_period = 1.0 / max(float(args.ros2_camera_rate), 1.0e-6)
    camera_width = max(int(args.ros2_camera_width), 1)
    camera_height = max(int(args.ros2_camera_height), 1)

    front_camera = _safe_get_attr(scene_cfg, "front_camera", None)
    if args.enable_ros2_depth and front_camera is not None:
        front_camera.update_period = camera_period
        front_camera.width = camera_width
        front_camera.height = camera_height
    else:
        scene_cfg.front_camera = None

    adas_camera = _safe_get_attr(scene_cfg, "adas_camera", None)
    if args.enable_ros2_auto_camera and adas_camera is not None:
        adas_camera.update_period = camera_period
        adas_camera.width = camera_width
        adas_camera.height = camera_height
    else:
        scene_cfg.adas_camera = None

    imu = _safe_get_attr(scene_cfg, "imu", None)
    if args.enable_ros2_imu and imu is not None:
        imu.update_period = 1.0 / max(float(args.ros2_imu_rate), 1.0e-6)
    else:
        scene_cfg.imu = None

    lidar = _safe_get_attr(scene_cfg, "lidar", None)
    if args.enable_ros2_mid360 and lidar is not None:
        lidar.update_period = 1.0 / max(float(args.ros2_mid360_rate), 1.0e-6)
    else:
        scene_cfg.lidar = None

    lidar_imu = _safe_get_attr(scene_cfg, "lidar_imu", None)
    if args.enable_ros2_mid360_imu and lidar_imu is not None:
        lidar_imu.update_period = 1.0 / max(float(args.ros2_imu_rate), 1.0e-6)
    else:
        scene_cfg.lidar_imu = None

    if not args.enable_ros2_height_map:
        for sensor_name in ("height_scanner", "base_height_scanner"):
            if hasattr(scene_cfg, sensor_name):
                setattr(scene_cfg, sensor_name, None)

    print(
        "[SENSORS] "
        f"front_camera={'on' if args.enable_ros2_depth else 'off'} "
        f"adas_camera={'on' if args.enable_ros2_auto_camera else 'off'} "
        f"camera={camera_width}x{camera_height}@{float(args.ros2_camera_rate):.1f}Hz "
        f"imu={'on' if args.enable_ros2_imu else 'off'}@{float(args.ros2_imu_rate):.1f}Hz "
        f"lidar={'on' if args.enable_ros2_mid360 else 'off'}@{float(args.ros2_mid360_rate):.1f}Hz "
        f"lidar_imu={'on' if args.enable_ros2_mid360_imu else 'off'}"
    )


def _set_obs_term_scale(group, term_name: str, value: float):
    term = _safe_get_attr(group, term_name, None)
    if term is not None and hasattr(term, "scale"):
        term.scale = float(value)


def _set_command_obs_scale(group, value):
    term = _safe_get_attr(group, "velocity_commands", None)
    params = _safe_get_attr(term, "params", None)
    if isinstance(params, dict):
        params["scale"] = tuple(float(v) for v in value)


def _configure_policy_runtime_settings(env_cfg, args):
    scene_cfg = _safe_get_attr(env_cfg, "scene", None)
    robot_cfg = _safe_get_attr(scene_cfg, "robot", None)
    actuators = _safe_get_attr(robot_cfg, "actuators", None)
    if isinstance(actuators, dict):
        shoulder = actuators.get("joints")
        if shoulder is not None:
            shoulder.stiffness = {".*_shoulder_joint": float(args.hw_shoulder_kp)}
            shoulder.damping = {".*_shoulder_joint": float(args.hw_shoulder_kd)}
        wheels = actuators.get("wheels")
        if wheels is not None:
            wheels.stiffness = {".*_wheel_joint": float(args.hw_wheel_kp)}
            wheels.damping = {".*_wheel_joint": float(args.hw_wheel_kd)}

    actions_cfg = _safe_get_attr(env_cfg, "actions", None)
    if actions_cfg is not None:
        joint_pos_action = _safe_get_attr(actions_cfg, "joint_pos", None)
        if joint_pos_action is not None:
            joint_pos_action.scale = float(args.action_shoulder_scale)
        wheel_vel_action = _safe_get_attr(actions_cfg, "wheel_vel", None)
        if wheel_vel_action is not None:
            wheel_vel_action.scale = float(args.action_wheel_scale)

    observations_cfg = _safe_get_attr(env_cfg, "observations", None)
    cmd_vel_scale = (
        float(args.obs_cmd_vel_x_scale),
        float(args.obs_cmd_vel_y_scale),
        float(args.obs_cmd_vel_yaw_scale),
    )
    if observations_cfg is not None:
        for group_name in ("stack_policy", "stack_critic"):
            group = _safe_get_attr(observations_cfg, group_name, None)
            _set_obs_term_scale(group, "joint_pos", args.obs_joint_pos_scale)
            _set_obs_term_scale(group, "joint_vel", args.obs_joint_vel_scale)
            _set_obs_term_scale(group, "base_ang_vel", args.obs_base_ang_vel_scale)
            _set_obs_term_scale(group, "base_projected_gravity", args.obs_projected_gravity_scale)
            _set_obs_term_scale(group, "projected_gravity", args.obs_projected_gravity_scale)
        for group_name in ("none_stack_policy", "none_stack_critic"):
            group = _safe_get_attr(observations_cfg, group_name, None)
            _set_command_obs_scale(group, cmd_vel_scale)

    print(
        "[POLICY_RUNTIME] "
        f"shoulder_kp={float(args.hw_shoulder_kp):.4g} shoulder_kd={float(args.hw_shoulder_kd):.4g} "
        f"wheel_kp={float(args.hw_wheel_kp):.4g} wheel_kd={float(args.hw_wheel_kd):.4g} "
        f"action_shoulder={float(args.action_shoulder_scale):.4g} action_wheel={float(args.action_wheel_scale):.4g} "
        f"obs_joint_pos={float(args.obs_joint_pos_scale):.4g} "
        f"obs_joint_vel={float(args.obs_joint_vel_scale):.4g} "
        f"obs_base_ang={float(args.obs_base_ang_vel_scale):.4g} "
        f"obs_projected_gravity={float(args.obs_projected_gravity_scale):.4g} "
        f"obs_cmd_vel={cmd_vel_scale}"
    )


def _get_robot_for_logging(env):
    env_unwrapped = _unwrap_env(env)
    scene = _safe_get_attr(env_unwrapped, "scene", None)
    if scene is None:
        return None
    try:
        return scene["robot"]
    except Exception:
        return None


def _get_scene_sensor(env, sensor_name: str):
    env_unwrapped = _unwrap_env(env)
    scene = _safe_get_attr(env_unwrapped, "scene", None)
    if scene is None:
        return None

    sensors = _safe_get_attr(scene, "sensors", None)
    if sensors is not None:
        try:
            return sensors[sensor_name]
        except Exception:
            pass

    try:
        return scene[sensor_name]
    except Exception:
        return None


def _get_robot_joint_names(robot) -> list[str]:
    joint_names = _safe_get_attr(robot, "joint_names", None)
    if joint_names is not None:
        return list(joint_names)

    cfg_joint_pos = _safe_get_attr(_safe_get_attr(_safe_get_attr(robot, "cfg", None), "init_state", None), "joint_pos", None)
    if isinstance(cfg_joint_pos, dict):
        return list(cfg_joint_pos.keys())

    joint_pos = _safe_get_attr(_safe_get_attr(robot, "data", None), "joint_pos", None)
    if isinstance(joint_pos, torch.Tensor):
        return [f"joint_{i}" for i in range(joint_pos.shape[-1])]
    return []


def _record_joint_history(robot, time_log: list[float], torque_log: list[np.ndarray], vel_log: list[np.ndarray], time_s: float):
    if robot is None:
        return

    robot_data = _safe_get_attr(robot, "data", None)
    applied_torque = _safe_get_attr(robot_data, "applied_torque", None)
    joint_vel = _safe_get_attr(robot_data, "joint_vel", None)
    if not isinstance(applied_torque, torch.Tensor) or not isinstance(joint_vel, torch.Tensor):
        return
    if applied_torque.shape[0] == 0 or joint_vel.shape[0] == 0:
        return

    time_log.append(float(time_s))
    torque_log.append(applied_torque[0].detach().cpu().numpy().copy())
    vel_log.append(joint_vel[0].detach().cpu().numpy().copy())


def _plot_joint_history(time_log: list[float], torque_log: list[np.ndarray], vel_log: list[np.ndarray], joint_names: list[str], log_dir: str):
    if len(time_log) == 0 or len(torque_log) == 0 or len(vel_log) == 0:
        print("[PLOT] No joint history collected. Skipping plots.")
        return

    times = np.asarray(time_log, dtype=np.float32)
    torques = np.asarray(torque_log, dtype=np.float32)
    velocities = np.asarray(vel_log, dtype=np.float32)

    num_joints = min(torques.shape[1], velocities.shape[1])
    if num_joints == 0:
        print("[PLOT] Joint history is empty. Skipping plots.")
        return

    if len(joint_names) < num_joints:
        joint_names = joint_names + [f"joint_{i}" for i in range(len(joint_names), num_joints)]
    else:
        joint_names = joint_names[:num_joints]

    cols = 2 if num_joints > 1 else 1
    rows = math.ceil(num_joints / cols)

    def _make_figure(values: np.ndarray, ylabel: str, title: str, output_name: str):
        fig, axes = plt.subplots(rows, cols, figsize=(14, max(4, rows * 2.8)), sharex=True)
        axes = np.atleast_1d(axes).reshape(-1)

        for idx in range(rows * cols):
            ax = axes[idx]
            if idx >= num_joints:
                ax.axis("off")
                continue
            ax.plot(times, values[:, idx], linewidth=1.2)
            ax.set_title(joint_names[idx], fontsize=10)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            if idx >= (rows - 1) * cols:
                ax.set_xlabel("time [s]")

        fig.suptitle(title)
        fig.tight_layout()
        output_path = os.path.join(log_dir, output_name)
        fig.savefig(output_path, dpi=150)
        print(f"[PLOT] saved: {output_path}")
        return fig

    _make_figure(torques[:, :num_joints], "torque [Nm]", "Joint Torque vs Time", "play_joint_torque.png")
    _make_figure(velocities[:, :num_joints], "velocity [rad/s]", "Joint Velocity vs Time", "play_joint_velocity.png")
    plt.show()


def main():
    """Play with CO-RL agent."""
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    if args_cli.teleop:
        _disable_terminations_for_teleop(env_cfg)
    _configure_policy_runtime_settings(env_cfg, args_cli)
    _configure_runtime_sensors(env_cfg, args_cli)

    agent_cfg: CoRlPolicyRunnerCfg = cli_args.parse_co_rl_cfg(args_cli.task, args_cli)
    agent_cfg.num_policy_stacks = (
        args_cli.num_policy_stacks if args_cli.num_policy_stacks is not None else agent_cfg.num_policy_stacks
    )
    agent_cfg.num_critic_stacks = (
        args_cli.num_critic_stacks if args_cli.num_critic_stacks is not None else agent_cfg.num_critic_stacks
    )

    is_off_policy = False if agent_cfg.to_dict()["algorithm"]["class_name"] in ["PPO", "SRMPPO"] else True

    policy_onnx_path = args_cli.policy_onnx_path.strip()
    if policy_onnx_path:
        policy_onnx_path = os.path.abspath(os.path.expanduser(policy_onnx_path))
        if not os.path.isfile(policy_onnx_path):
            raise FileNotFoundError(f"ONNX policy file does not exist: {policy_onnx_path}")
        resume_path = None
        log_dir = os.path.dirname(policy_onnx_path)
        print(f"[INFO] Loading ONNX policy from: {policy_onnx_path}")
    else:
        log_root_path = os.path.join("logs", "co_rl", agent_cfg.experiment_name, args_cli.algo)
        log_root_path = os.path.abspath(log_root_path)
        print(f"[INFO] Loading experiment from directory: {log_root_path}")

        if args_cli.use_pretrained_checkpoint:
            resume_path = get_published_pretrained_checkpoint("co_rl", args_cli.task)
            if not resume_path:
                print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
                return
        else:
            resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

        log_dir = os.path.dirname(resume_path)

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    _hide_isaac_debug_visuals()
    _sanitize_reflective_surface_materials()

    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    if isinstance(env.unwrapped, ManagerBasedConstraintRLEnv):
        agent_cfg.use_constraint_rl = True

    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    if args_cli.analyze is not None:
        analyze_items = args_cli.analyze[0].split()
        analyzer = Analyzer(env=env, analyze_items=analyze_items, log_dir=log_dir)

    env = CoRlVecEnvWrapper(env, agent_cfg)

    runner = None
    srm = None
    if policy_onnx_path:
        policy = OnnxPolicy(policy_onnx_path, env.unwrapped.device)
    else:
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        if is_off_policy:
            runner = OffPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        else:
            if args_cli.algo == "srmppo":
                runner = SRMOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
            elif args_cli.algo == "ppo":
                runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
            else:
                raise ValueError(f"Unsupported algo: {args_cli.algo}")

        runner.load(resume_path)

        if hasattr(runner.alg, "srm") and hasattr(runner.alg, "srm_fc"):
            srm = runner.alg.srm

        policy = runner.get_inference_policy(device=env.unwrapped.device)

    front_camera_publisher = None
    if args_cli.enable_ros2_depth:
        front_camera_publisher = create_front_camera_publisher(
            rgb_topic=args_cli.ros2_front_rgb_topic,
            depth_topic=args_cli.ros2_depth_topic,
            camera_info_topic=args_cli.ros2_camera_info_topic,
            frame_id=args_cli.ros2_depth_frame_id,
        )
        print(
            "[INFO] Publishing ROS 2 front camera RGB/depth from env_0 F_camera_link "
            f"to '{args_cli.ros2_front_rgb_topic}' and '{args_cli.ros2_depth_topic}'"
        )
    adas_camera_publisher = None
    if args_cli.enable_ros2_auto_camera:
        adas_camera_publisher = create_front_camera_publisher(
            rgb_topic=args_cli.ros2_auto_rgb_topic,
            depth_topic=args_cli.ros2_auto_depth_topic,
            camera_info_topic=args_cli.ros2_auto_depth_camera_info_topic,
            frame_id=args_cli.ros2_auto_depth_frame_id,
        )
        print(
            "[INFO] Publishing ROS 2 ADAS camera RGB/depth from env_0 A_camera_link "
            f"to '{args_cli.ros2_auto_depth_topic}' and '{args_cli.ros2_auto_rgb_topic}'"
        )
    if args_cli.enable_ros2_stereo:
        create_auto_stereo_camera_ros_graphs(
            left_image_topic=args_cli.ros2_auto_left_image_topic,
            right_image_topic=args_cli.ros2_auto_right_image_topic,
            left_camera_info_topic=args_cli.ros2_auto_left_camera_info_topic,
            right_camera_info_topic=args_cli.ros2_auto_right_camera_info_topic,
            left_frame_id=args_cli.ros2_auto_left_frame_id,
            right_frame_id=args_cli.ros2_auto_right_frame_id,
        )
        print(
            "[INFO] Publishing ROS 2 A_camera_link virtual D435i stereo streams from env_0 "
            f"to '{args_cli.ros2_auto_left_image_topic}' and '{args_cli.ros2_auto_right_image_topic}'"
        )

    clock_publisher = create_clock_publisher(topic_name="/clock")
    print("[INFO] Publishing authoritative ROS 2 simulation clock to '/clock'")

    time_publisher = create_time_ros_graph(topic_name=args_cli.ros2_time_topic)
    print(f"[INFO] Publishing simulation time to '{args_cli.ros2_time_topic}'")

    imu_publisher = None
    if args_cli.enable_ros2_imu:
        imu_publisher = create_imu_ros_graph(
            topic_name=args_cli.ros2_imu_topic,
            frame_id=args_cli.ros2_imu_frame_id,
        )
        print(f"[INFO] Publishing ROS 2 IMU from env_0 base_link to '{args_cli.ros2_imu_topic}'")

    front_camera_imu_publisher = None
    if args_cli.enable_ros2_front_camera_imu:
        front_camera_imu_publisher = create_imu_ros_graph(
            graph_path="/ROS_FrontCameraImu",
            topic_name=args_cli.ros2_front_camera_imu_topic,
            frame_id=args_cli.ros2_front_camera_imu_frame_id,
        )
        print(
            "[INFO] Publishing ROS 2 front camera IMU from env_0 F_camera_link "
            f"to '{args_cli.ros2_front_camera_imu_topic}'"
        )

    mid360_imu_publisher = None
    if args_cli.enable_ros2_mid360_imu:
        mid360_imu_publisher = create_imu_ros_graph(
            graph_path="/ROS_Mid360Imu",
            topic_name=args_cli.ros2_mid360_imu_topic,
            frame_id=args_cli.ros2_mid360_frame_id,
            derive_angular_velocity=True,
        )
        print(f"[INFO] Publishing ROS 2 lidar IMU to '{args_cli.ros2_mid360_imu_topic}'")

    robot_state_publisher = None
    if args_cli.enable_ros2_robot_state:
        robot_state_publisher = create_robot_state_publisher(
            joint_states_topic=args_cli.ros2_joint_states_topic,
            world_frame_id=args_cli.ros2_world_frame_id,
            base_frame_id=args_cli.ros2_base_frame_id,
            path_gt_topic=args_cli.ros2_path_gt_topic,
            gt_odom_topic=args_cli.ros2_gt_odom_topic,
            gt_child_frame_id=args_cli.ros2_gt_child_frame_id,
            path_gt_max_poses=args_cli.ros2_path_gt_max_poses,
            publish_root_tf=args_cli.ros2_publish_root_tf,
            relative_to_initial_pose=args_cli.ros2_path_gt_relative_to_initial,
            diagnostics_csv_path=args_cli.ros2_slam_diagnostics_csv,
        )
        print(
            "[INFO] Publishing ROS 2 robot state for RViz "
            f"('{args_cli.ros2_joint_states_topic}', '{args_cli.ros2_path_gt_topic}')"
        )

    height_map_publisher = None
    if args_cli.enable_ros2_height_map:
        height_map_publisher = create_height_map_pointcloud_publisher(
            topic_name=args_cli.ros2_height_map_topic,
            frame_id=args_cli.ros2_height_map_frame_id,
        )
        print(
            "[INFO] Publishing ROS 2 height map PointCloud2 from env_0 "
            f"sensor '{args_cli.ros2_height_map_sensor}' to '{args_cli.ros2_height_map_topic}'"
        )

    mid360_lidar_publisher = None
    if args_cli.enable_ros2_mid360:
        if args_cli.enable_ros2_mid360_rtx:
            create_mid360_rtx_lidar_ros_graph(
                topic_name=args_cli.ros2_mid360_lidar_topic,
                frame_id=args_cli.ros2_mid360_frame_id,
            )
            print(
                "[INFO] Publishing ROS 2 RTX lidar PointCloud2 from env_0 "
                f"to '{args_cli.ros2_mid360_lidar_topic}'"
            )
        else:
            mid360_lidar_publisher = create_mid360_pointcloud_publisher(
                topic_name=args_cli.ros2_mid360_lidar_topic,
                frame_id=args_cli.ros2_mid360_frame_id,
            )
            print(
                "[INFO] Publishing ROS 2 RayCaster lidar PointCloud2 from env_0 "
                f"sensor '{args_cli.ros2_mid360_sensor}' to '{args_cli.ros2_mid360_lidar_topic}'"
            )

    if runner is not None:
        export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
        if is_off_policy:
            export_policy_as_jit(runner.alg, runner.obs_normalizer, path=export_model_dir, filename="policy.pt")
            export_policy_as_onnx(
                runner.alg, normalizer=runner.obs_normalizer, path=export_model_dir, filename="policy.onnx"
            )
        else:
            export_policy_as_jit(
                runner.alg.actor_critic, runner.obs_normalizer, path=export_model_dir, filename="policy.pt"
            )
            export_policy_as_onnx(
                runner.alg.actor_critic, normalizer=runner.obs_normalizer, path=export_model_dir, filename="policy.onnx"
            )
            if args_cli.algo == "srmppo":
                export_srm_as_onnx(
                    runner.alg.srm, runner.alg.srm_fc, device=agent_cfg.device, path=export_model_dir, filename="srm.onnx"
                )

        export_env_as_pdf(
            yaml_path=os.path.join(log_dir, "params", "env.yaml"),
            pdf_path=os.path.join(export_model_dir, "env.pdf")
        )

    obs, _ = env.get_observations()
    env_dt = _get_env_dt(_unwrap_env(env))
    robot = _get_robot_for_logging(env)
    front_camera_sensor = _get_scene_sensor(env, "front_camera")
    adas_camera_sensor = _get_scene_sensor(env, "adas_camera")
    imu_sensor = _get_scene_sensor(env, "imu")
    front_camera_imu_sensor = _get_scene_sensor(env, "front_camera_imu")
    mid360_imu_sensor = _get_scene_sensor(env, args_cli.ros2_mid360_imu_sensor)
    height_map_sensor = _get_scene_sensor(env, args_cli.ros2_height_map_sensor)
    mid360_lidar_sensor = _get_scene_sensor(env, args_cli.ros2_mid360_sensor)
    joint_names = _get_robot_joint_names(robot)
    time_log: list[float] = []
    torque_log: list[np.ndarray] = []
    vel_log: list[np.ndarray] = []
    sim_time_s = 0.0
    ros_publish_period_s = 1.0 / 50.0
    next_ros_publish_time_s = 0.0
    requested_imu_publish_period_s = 1.0 / max(float(args_cli.ros2_imu_rate), 1.0e-6)
    # Sensor tensors are sampled once per environment step. Publishing the
    # same tensor multiple times with invented intermediate stamps corrupts
    # inertial integration, so cap ROS IMU output at the actual sample rate.
    imu_publish_period_s = max(requested_imu_publish_period_s, env_dt)
    next_imu_publish_time_s = imu_publish_period_s
    if requested_imu_publish_period_s < env_dt:
        print(
            f"[INFO] Capping ROS 2 IMU rate at the simulation sample rate "
            f"({1.0 / env_dt:.1f} Hz; requested {args_cli.ros2_imu_rate:.1f} Hz)."
        )
    camera_publish_period_s = 1.0 / max(float(args_cli.ros2_camera_rate), 1.0e-6)
    next_camera_publish_time_s = 0.0
    mid360_publish_period_s = 1.0 / max(float(args_cli.ros2_mid360_rate), 1.0e-6)
    next_mid360_publish_time_s = 0.0
    command_user_publish_period_s = 1.0 / max(float(args_cli.ros2_command_user_rate), 1.0e-6)
    next_command_user_publish_time_s = 0.0
    perf_wall_start_s = time.perf_counter()
    perf_sim_start_s = sim_time_s
    next_perf_report_wall_s = perf_wall_start_s + max(float(args_cli.perf_report_interval), 0.0)

    teleop = None
    teleop_spec = None
    command_user_bridge = None

    if args_cli.teleop or args_cli.enable_ros2_command_user:
        teleop_spec = infer_teleop_command_spec(env, obs)
        env_unwrapped = _unwrap_env(env)
        dt = _get_env_dt(env_unwrapped)
        print(f"[TELEOP] env dt         : {dt}")

    if args_cli.teleop:
        print(f"[TELEOP] teleop rate    : {args_cli.teleop_delta} per second")
        teleop = KeyboardTeleop(
            num_envs=env.num_envs,
            device=env.unwrapped.device,
            spec=teleop_spec,
            delta=args_cli.teleop_delta,
            dt=dt,
            use_stdin=args_cli.teleop_use_stdin,
        )
        applied = teleop.apply_to_env(env)
        print(f"[TELEOP] env command sync: {'enabled' if applied else 'fallback(obs only)'}")

    if args_cli.enable_ros2_command_user:
        command_user_bridge = create_command_user_bridge(
            topic_name=args_cli.ros2_command_user_topic,
            command_dim=teleop_spec.command_dim,
            axis_names=teleop_spec.axis_names,
            child_frame_id=args_cli.ros2_base_frame_id,
            cmd_vel_topic=args_cli.ros2_nav_cmd_vel_topic,
        )
        print(
            "[INFO] Bridging ROS 2 CommandUser "
            f"('{args_cli.ros2_command_user_topic}') and Nav2 Twist "
            f"('{args_cli.ros2_nav_cmd_vel_topic}') to command term '{teleop_spec.command_term_name}'"
        )
        if args_cli.teleop and args_cli.ros2_command_user_publish_keyboard:
            print("[INFO] Keyboard teleop commands are published as core/msg/CommandUser before applying to sim.")

    if args_cli.enable_ros2_depth and front_camera_sensor is None:
        print("[WARN] ROS 2 front camera publishing requested, but scene sensor 'front_camera' was not found.")
    if args_cli.enable_ros2_auto_camera and adas_camera_sensor is None:
        print("[WARN] ROS 2 ADAS camera publishing requested, but scene sensor 'adas_camera' was not found.")
    if args_cli.enable_ros2_imu and imu_sensor is None:
        print("[WARN] ROS 2 IMU publishing requested, but scene sensor 'imu' was not found.")
    if args_cli.enable_ros2_front_camera_imu and front_camera_imu_sensor is None:
        print(
            "[WARN] ROS 2 front camera IMU publishing requested, "
            "but scene sensor 'front_camera_imu' was not found."
        )
    if args_cli.enable_ros2_height_map and height_map_sensor is None:
        print(
            "[WARN] ROS 2 height map publishing requested, "
            f"but scene sensor '{args_cli.ros2_height_map_sensor}' was not found."
        )
    if args_cli.enable_ros2_mid360 and not args_cli.enable_ros2_mid360_rtx and mid360_lidar_sensor is None:
        print(
            "[WARN] ROS 2 lidar publishing requested, "
            f"but scene sensor '{args_cli.ros2_mid360_sensor}' was not found."
        )
    if args_cli.enable_ros2_mid360_imu and mid360_imu_sensor is None:
        print(
            "[WARN] ROS 2 lidar IMU publishing requested, "
            f"but scene sensor '{args_cli.ros2_mid360_imu_sensor}' was not found."
        )

    timestep = 0
    try:
        while simulation_app.is_running():
            with torch.inference_mode():
                if command_user_bridge is not None:
                    if teleop is not None and args_cli.ros2_command_user_publish_keyboard:
                        teleop.update()
                        if sim_time_s + 1.0e-9 >= next_command_user_publish_time_s:
                            command_user_bridge.publish(
                                teleop.current_command,
                                timestamp_s=sim_time_s,
                                dt=command_user_publish_period_s,
                            )
                            next_command_user_publish_time_s += command_user_publish_period_s
                        else:
                            command_user_bridge.spin_once()
                        command_tensor = _command_tensor_from_array(
                            command_user_bridge.get_latest_command(),
                            teleop_spec,
                            env.num_envs,
                            env.unwrapped.device,
                        )
                    else:
                        command_user_bridge.spin_once()
                        command_tensor = _command_tensor_from_array(
                            command_user_bridge.get_latest_command(),
                            teleop_spec,
                            env.num_envs,
                            env.unwrapped.device,
                        )
                    _set_env_command(_unwrap_env(env), teleop_spec.command_term_name, command_tensor)
                    obs_for_policy = _apply_command_to_obs(obs, teleop_spec, command_tensor)

                    if srm is not None:
                        encoded_obs = runner.alg.encode_obs(obs_for_policy)
                        actions = policy(encoded_obs)
                    else:
                        actions = policy(obs_for_policy)
                elif args_cli.teleop:
                    teleop.update()
                    teleop.apply_to_env(env)
                    obs_for_policy = teleop.apply_to_obs(obs)

                    if srm is not None:
                        encoded_obs = runner.alg.encode_obs(obs_for_policy)
                        actions = policy(encoded_obs)
                    else:
                        actions = policy(obs_for_policy)
                else:
                    if srm is not None:
                        encoded_obs = runner.alg.encode_obs(obs)
                        actions = policy(encoded_obs)
                    else:
                        actions = policy(obs)

                obs, _, _, extras = env.step(actions)
                sim_time_s += env_dt
                _record_joint_history(robot, time_log, torque_log, vel_log, sim_time_s)
                if args_cli.perf_report_interval > 0.0:
                    wall_now_s = time.perf_counter()
                    if wall_now_s >= next_perf_report_wall_s:
                        wall_elapsed_s = max(wall_now_s - perf_wall_start_s, 1.0e-9)
                        sim_elapsed_s = sim_time_s - perf_sim_start_s
                        print(
                            f"[PERF] realtime_factor={sim_elapsed_s / wall_elapsed_s:.2f} "
                            f"sim_dt={env_dt:.4f}s ros_hz={1.0 / ros_publish_period_s:.1f} "
                            f"cmd_hz={args_cli.ros2_command_user_rate:.1f}"
                        )
                        perf_wall_start_s = wall_now_s
                        perf_sim_start_s = sim_time_s
                        next_perf_report_wall_s = wall_now_s + float(args_cli.perf_report_interval)

                if sim_time_s + 1.0e-9 >= next_ros_publish_time_s:
                    clock_publisher.publish(sim_time_s)
                    time_publisher.publish(sim_time_s)
                    if robot_state_publisher is not None and robot is not None:
                        robot_data = _safe_get_attr(robot, "data", None)
                        joint_pos = _safe_get_attr(robot_data, "joint_pos", None)
                        joint_vel = _safe_get_attr(robot_data, "joint_vel", None)
                        root_pos = _safe_get_attr(robot_data, "root_link_pos_w", None)
                        root_quat = _safe_get_attr(robot_data, "root_link_quat_w", None)
                        if root_pos is None:
                            root_pos = _safe_get_attr(robot_data, "root_pos_w", None)
                        if root_quat is None:
                            root_quat = _safe_get_attr(robot_data, "root_quat_w", None)

                        if (
                            isinstance(joint_pos, torch.Tensor)
                            and isinstance(joint_vel, torch.Tensor)
                            and isinstance(root_pos, torch.Tensor)
                            and isinstance(root_quat, torch.Tensor)
                            and joint_pos.shape[0] > 0
                            and joint_vel.shape[0] > 0
                            and root_pos.shape[0] > 0
                            and root_quat.shape[0] > 0
                        ):
                            publish_joint_names = joint_names[: joint_pos.shape[-1]]
                            if len(publish_joint_names) < joint_pos.shape[-1]:
                                publish_joint_names = publish_joint_names + [
                                    f"joint_{i}" for i in range(len(publish_joint_names), joint_pos.shape[-1])
                                ]
                            robot_state_publisher.publish(
                                joint_names=publish_joint_names,
                                joint_positions=joint_pos[0].detach().cpu().tolist(),
                                joint_velocities=joint_vel[0].detach().cpu().tolist(),
                                root_position_xyz=root_pos[0].detach().cpu().tolist(),
                                root_orientation_wxyz=root_quat[0].detach().cpu().tolist(),
                                timestamp_s=sim_time_s,
                                gt_position_xyz=(
                                    mid360_imu_sensor.data.pos_w[0].detach().cpu().tolist()
                                    if mid360_imu_sensor is not None
                                    else None
                                ),
                                gt_orientation_wxyz=(
                                    mid360_imu_sensor.data.quat_w[0].detach().cpu().tolist()
                                    if mid360_imu_sensor is not None
                                    else None
                                ),
                            )

                    if height_map_publisher is not None and height_map_sensor is not None:
                        height_map_data = _safe_get_attr(height_map_sensor, "data", None)
                        ray_hits_w = _safe_get_attr(height_map_data, "ray_hits_w", None)
                        if isinstance(ray_hits_w, torch.Tensor) and ray_hits_w.shape[0] > 0:
                            height_map_publisher.publish(ray_hits_w=ray_hits_w, timestamp_s=sim_time_s)

                    next_ros_publish_time_s += ros_publish_period_s

                while sim_time_s + 1.0e-9 >= next_imu_publish_time_s:
                    if imu_publisher is not None and imu_sensor is not None:
                        imu_data = imu_sensor.data
                        imu_publisher.publish(
                            orientation_wxyz=imu_data.quat_w[0].detach().cpu().tolist(),
                            angular_velocity_xyz=imu_data.ang_vel_b[0].detach().cpu().tolist(),
                            linear_acceleration_xyz=imu_data.lin_acc_b[0].detach().cpu().tolist(),
                            timestamp_s=next_imu_publish_time_s,
                        )

                    if front_camera_imu_publisher is not None and front_camera_imu_sensor is not None:
                        front_camera_imu_data = front_camera_imu_sensor.data
                        front_camera_imu_publisher.publish(
                            orientation_wxyz=front_camera_imu_data.quat_w[0].detach().cpu().tolist(),
                            angular_velocity_xyz=front_camera_imu_data.ang_vel_b[0].detach().cpu().tolist(),
                            linear_acceleration_xyz=front_camera_imu_data.lin_acc_b[0].detach().cpu().tolist(),
                            timestamp_s=next_imu_publish_time_s,
                        )

                    if mid360_imu_publisher is not None and mid360_imu_sensor is not None:
                        mid360_imu_data = mid360_imu_sensor.data
                        mid360_imu_publisher.publish(
                            orientation_wxyz=mid360_imu_data.quat_w[0].detach().cpu().tolist(),
                            angular_velocity_xyz=mid360_imu_data.ang_vel_b[0].detach().cpu().tolist(),
                            linear_acceleration_xyz=mid360_imu_data.lin_acc_b[0].detach().cpu().tolist(),
                            timestamp_s=next_imu_publish_time_s,
                        )
                    next_imu_publish_time_s += imu_publish_period_s

                if sim_time_s + 1.0e-9 >= next_camera_publish_time_s:
                    if front_camera_publisher is not None and front_camera_sensor is not None:
                        front_camera_publisher.publish(front_camera_sensor.data, timestamp_s=sim_time_s)
                    if adas_camera_publisher is not None and adas_camera_sensor is not None:
                        adas_camera_publisher.publish(adas_camera_sensor.data, timestamp_s=sim_time_s)
                    next_camera_publish_time_s += camera_publish_period_s

                if (
                    mid360_lidar_publisher is not None
                    and mid360_lidar_sensor is not None
                    and sim_time_s + 1.0e-9 >= next_mid360_publish_time_s
                ):
                    mid360_data = _safe_get_attr(mid360_lidar_sensor, "data", None)
                    ray_hits_w = _safe_get_attr(mid360_data, "ray_hits_w", None)
                    # RayCaster has an additional pattern-orientation offset.
                    # Express world hits in the physical lidar/IMU link frame,
                    # which is the frame declared in PointCloud2 and used by
                    # Super-LIO's identity lidar-to-IMU extrinsic.
                    lidar_pose_data = (
                        _safe_get_attr(mid360_imu_sensor, "data", None)
                        if mid360_imu_sensor is not None
                        else mid360_data
                    )
                    sensor_pos_w = _safe_get_attr(lidar_pose_data, "pos_w", None)
                    sensor_quat_w = _safe_get_attr(lidar_pose_data, "quat_w", None)
                    if (
                        isinstance(ray_hits_w, torch.Tensor)
                        and isinstance(sensor_pos_w, torch.Tensor)
                        and isinstance(sensor_quat_w, torch.Tensor)
                        and ray_hits_w.shape[0] > 0
                    ):
                        mid360_lidar_publisher.publish(
                            ray_hits_w=ray_hits_w,
                            timestamp_s=sim_time_s,
                            sensor_pos_w=sensor_pos_w[0],
                            sensor_quat_wxyz=sensor_quat_w[0],
                        )
                    next_mid360_publish_time_s += mid360_publish_period_s

            if args_cli.video:
                timestep += 1
                if timestep == args_cli.video_length:
                    break

            if args_cli.analyze is not None:
                analyzer.append(extras["observations"]["obs_info"])
    finally:
        if teleop is not None:
            teleop.stop()
        if robot_state_publisher is not None:
            robot_state_publisher.close()
        if height_map_publisher is not None:
            height_map_publisher.close()
        if mid360_lidar_publisher is not None:
            mid360_lidar_publisher.close()
        if front_camera_publisher is not None:
            front_camera_publisher.close()
        if adas_camera_publisher is not None:
            adas_camera_publisher.close()
        if command_user_bridge is not None:
            command_user_bridge.close()
        if clock_publisher is not None:
            clock_publisher.close()
        if time_publisher is not None:
            time_publisher.close()
        env.close()

    if args_cli.analyze is not None:
        analyzer.export()

    _plot_joint_history(time_log, torque_log, vel_log, joint_names, log_dir)


if __name__ == "__main__":
    main()
    simulation_app.close()
