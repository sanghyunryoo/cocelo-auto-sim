from __future__ import annotations

import torch
from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING
from isaaclab.utils import configclass

from isaaclab.envs.mdp import UniformVelocityCommand
from isaaclab.envs.mdp.commands.commands_cfg import UniformVelocityCommandCfg


if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class UniformVelocityWithZCommand(UniformVelocityCommand):
    r"""Command generator that generates a velocity command in SE(2) with an additional position command in Z.

    The command comprises of a linear velocity in x and y direction, an angular velocity around
    the z-axis, and a position command in z. It is given in the robot's base frame.
    """

    def __init__(self, cfg: "UniformVelocityWithZCommandCfg", env: "ManagerBasedEnv"):
        super().__init__(cfg, env)
        self.vel_command_b = torch.zeros(self.num_envs, 4, device=self.device)
        self.time_elapsed = torch.zeros(self.num_envs, device=self.device)
        self.initial_choice = None
        self.track_z_flag = (cfg.ranges.pos_z[0] != 0.0 or cfg.ranges.pos_z[1] != 0.0)

    def __str__(self) -> str:
        msg = super().__str__()
        msg += f"\n\tPosition z range: {self.cfg.ranges.pos_z}"
        msg += f"\n\tPosition z num categories: {self.cfg.pos_z_num_categories}"
        return msg

    @property
    def command(self) -> torch.Tensor:
        return self.vel_command_b

    def _update_metrics(self):
        max_command_time = self.cfg.resampling_time_range[1]
        max_command_step = max_command_time / self._env.step_dt
        self.metrics["error_vel_xy"] += (
            torch.norm(self.vel_command_b[:, :2] - self.robot.data.root_link_lin_vel_b[:, :2], dim=-1) / max_command_step
        )
        self.metrics["error_vel_yaw"] += (
            torch.abs(self.vel_command_b[:, 2] - self.robot.data.root_link_ang_vel_b[:, 2]) / max_command_step
        )

    def _resample_command(self, env_ids: Sequence[int]):
        r = torch.empty(len(env_ids), device=self.device)
        self.vel_command_b[env_ids, 0] = r.uniform_(*self.cfg.ranges.lin_vel_x)
        self.vel_command_b[env_ids, 1] = r.uniform_(*self.cfg.ranges.lin_vel_y)
        self.vel_command_b[env_ids, 2] = r.uniform_(*self.cfg.ranges.ang_vel_z)

        if self.track_z_flag:
            self.vel_command_b[env_ids, 3] = self.gcd(env_ids, self.cfg.pos_z_num_categories)
        else:
            self.vel_command_b[env_ids, 3] = 0.0

        if self.cfg.heading_command:
            self.heading_target[env_ids] = r.uniform_(*self.cfg.ranges.heading)
            self.is_heading_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_heading_envs

        self.is_standing_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_standing_envs

        standing_env_ids = self.is_standing_env.nonzero(as_tuple=False).flatten()
        if len(standing_env_ids) > 0:
            if self.track_z_flag:
                self.standing_choice = self.gcd(standing_env_ids, 2)
            else:
                self.standing_choice = torch.zeros_like(standing_env_ids, dtype=torch.float32, device=self.device)

    def _update_command(self):
        self.time_elapsed += self._env.step_dt
        initial_phase_env_ids = (self.time_elapsed <= self.cfg.initial_phase_time).nonzero(as_tuple=False).flatten()
        if len(initial_phase_env_ids) > 0:
            self.vel_command_b[initial_phase_env_ids, :3] = 0.0
            if self.track_z_flag:
                self.vel_command_b[initial_phase_env_ids, 3] = self.gcd(
                    initial_phase_env_ids, self.cfg.pos_z_num_categories
                )
            else:
                self.vel_command_b[initial_phase_env_ids, 3] = 0.0

        reset_env_ids = self._env.reset_buf.nonzero(as_tuple=False).flatten()
        if len(reset_env_ids) > 0:
            self.time_elapsed[reset_env_ids] = 0.0

        standing_env_ids = self.is_standing_env.nonzero(as_tuple=False).flatten()
        self.vel_command_b[standing_env_ids, :3] = 0.0
        if len(standing_env_ids) > 0:
            self.vel_command_b[standing_env_ids, 3] = self.standing_choice

    def gcd(self, env_ids: Sequence[int], num_categories: int) -> torch.Tensor:
        if len(env_ids) == 0:
            return torch.tensor([], device=self.device)

        probabilities = torch.ones(num_categories, device=self.device) / num_categories
        categories = torch.linspace(
            self.cfg.ranges.pos_z[0], self.cfg.ranges.pos_z[1], num_categories, device=self.device
        )
        return categories[torch.multinomial(probabilities, len(env_ids), replacement=True)]

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return
        base_pos_w = self.robot.data.root_link_pos_w.clone()
        base_pos_w[:, 2] += 1.0
        vel_des_arrow_scale, vel_des_arrow_quat = self._resolve_xy_velocity_to_arrow(self.command[:, :2])
        vel_arrow_scale, vel_arrow_quat = self._resolve_xy_velocity_to_arrow(self.robot.data.root_link_lin_vel_b[:, :2])
        self.goal_vel_visualizer.visualize(base_pos_w, vel_des_arrow_quat, vel_des_arrow_scale)
        self.current_vel_visualizer.visualize(base_pos_w, vel_arrow_quat, vel_arrow_scale)


@configclass
class UniformVelocityWithZCommandCfg(UniformVelocityCommandCfg):
    """Configuration for the uniform velocity with z command generator."""

    class_type: type = UniformVelocityWithZCommand

    @configclass
    class Ranges(UniformVelocityCommandCfg.Ranges):
        pos_z: tuple[float, float] = MISSING  # min max [m]

    ranges: Ranges = MISSING
    initial_phase_time: float = 2.0
    pos_z_num_categories: int = 2
