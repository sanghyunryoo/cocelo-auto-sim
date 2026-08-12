# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
import numpy as np
import omni.log
import omni.usd
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg, ImuCfg, RayCasterCfg, patterns
from isaaclab.sensors.ray_caster.ray_caster import RayCaster
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.utils.noise import GaussianNoiseCfg as Gnoise
from isaaclab.utils.warp import convert_to_warp_mesh
from pxr import Usd, UsdGeom
import lab.flamingo.tasks.manager_based.locomotion.velocity.mdp as mdp
from lab.flamingo.tasks.manager_based.locomotion.velocity.terrain_config.stair_config import ROUGH_TERRAINS_CFG


class MergedStaticMeshRayCaster(RayCaster):
    """RayCaster that merges all Mesh descendants under each configured root prim."""

    def _initialize_warp_meshes(self):
        for mesh_root_path in self.cfg.mesh_prim_paths:
            stage = sim_utils.get_current_stage()
            root_prim = stage.GetPrimAtPath(mesh_root_path)
            if not root_prim.IsValid():
                raise RuntimeError(f"Invalid mesh root path for ray-casting: {mesh_root_path}")

            deinstanced_count = 0
            for prim in Usd.PrimRange(root_prim):
                if prim.IsInstanceable():
                    prim.SetInstanceable(False)
                    deinstanced_count += 1

            mesh_prims = [prim for prim in Usd.PrimRange(root_prim) if prim.GetTypeName() == "Mesh"]
            if not mesh_prims:
                raise RuntimeError(f"No mesh descendants found for ray-casting under: {mesh_root_path}")

            all_points = []
            all_triangles = []
            vertex_offset = 0
            skipped_meshes = 0

            for prim in mesh_prims:
                mesh = UsdGeom.Mesh(prim)
                points_attr = mesh.GetPointsAttr().Get()
                indices_attr = mesh.GetFaceVertexIndicesAttr().Get()
                counts_attr = mesh.GetFaceVertexCountsAttr().Get()
                if points_attr is None or indices_attr is None or counts_attr is None:
                    skipped_meshes += 1
                    continue

                points = np.asarray(points_attr, dtype=np.float32)
                if points.size == 0:
                    skipped_meshes += 1
                    continue
                transform_matrix = np.array(omni.usd.get_world_transform_matrix(mesh)).T
                points = np.matmul(points, transform_matrix[:3, :3].T)
                points += transform_matrix[:3, 3]

                local_indices = np.asarray(indices_attr, dtype=np.int32)
                counts = np.asarray(counts_attr, dtype=np.int32)
                triangles = self._triangulate_faces(local_indices, counts)
                if triangles.size == 0:
                    skipped_meshes += 1
                    continue

                all_points.append(points)
                all_triangles.append(triangles + vertex_offset)
                vertex_offset += points.shape[0]

            if not all_points or not all_triangles:
                raise RuntimeError(f"No valid mesh triangles found for ray-casting under: {mesh_root_path}")

            merged_points = np.concatenate(all_points, axis=0)
            merged_triangles = np.concatenate(all_triangles, axis=0)
            self.meshes[mesh_root_path] = convert_to_warp_mesh(merged_points, merged_triangles, device=self.device)
            omni.log.info(
                f"Merged {len(all_points)} mesh prims under {mesh_root_path} for ray-casting "
                f"({merged_points.shape[0]} vertices, {merged_triangles.shape[0]} triangles, "
                f"{skipped_meshes} skipped, {deinstanced_count} de-instanced)."
            )

    @staticmethod
    def _triangulate_faces(indices: np.ndarray, counts: np.ndarray) -> np.ndarray:
        triangles = []
        cursor = 0
        for count in counts:
            face = indices[cursor : cursor + count]
            cursor += count
            if count < 3:
                continue
            if count == 3:
                triangles.append(face)
                continue
            for i in range(1, count - 1):
                triangles.append(np.array([face[0], face[i], face[i + 1]], dtype=np.int32))
        if not triangles:
            return np.empty((0, 3), dtype=np.int32)
        return np.asarray(triangles, dtype=np.int32).reshape(-1, 3)


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=ROUGH_TERRAINS_CFG,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/Base/Architecture/Shingles_01.mdl",
            project_uvw=True,
        ),
        debug_vis=False,
    )
    #! ****************** Robots *************** !#
    robot: ArticulationCfg = MISSING
    #! ****************** Sensors ************** !#
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.07, size=[0.8, 0.8]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )
    base_height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.025, 0.025]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )
    front_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/F_camera_link/front_camera",
        update_period=1.0 / 30.0,
        height=240,
        width=320,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 100.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0), convention="ros"),
    )
    adas_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/A_camera_link/adas_camera",
        update_period=1.0 / 30.0,
        height=240,
        width=320,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 100.0),
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0), convention="ros"),
    )
    imu = ImuCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        update_period=1.0 / 100.0,
    )
    lidar = RayCasterCfg(
        class_type=MergedStaticMeshRayCaster,
        prim_path="{ENV_REGEX_NS}/Robot/lidar_link",
        update_period=1.0 / 5.0,
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(0.0, 1.0, 0.0, 0.0)),
        mesh_prim_paths=["/World/ground"],
        ray_alignment="base",
        max_distance=15.0,
        pattern_cfg=patterns.LidarPatternCfg(
            channels=16,
            vertical_fov_range=(-16.0, 16.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=1.0,
        ),
        debug_vis=False,
    )
    lidar_imu = ImuCfg(
        prim_path="{ENV_REGEX_NS}/Robot/lidar_link",
        update_period=1.0 / 100.0,
    )
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    #! ****************** Lights ************** !#
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(
            color=(0.75, 0.75, 0.75), intensity=4000.0
        ), 
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            color=(0.53, 0.81, 0.98), intensity=1500.0
        ),  
    )

@configclass
class CommandsCfg:
    """Command specifications for the MDP."""
    base_velocity = mdp.UniformVelocityWithZCommandCfg(
        asset_name="robot",
        resampling_time_range=(6.0, 8.0),
        rel_standing_envs=0.01,
        rel_heading_envs=0.0,
        heading_command=False,
        debug_vis=False,
        ranges=mdp.UniformVelocityWithZCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0), lin_vel_y=(-0.0, 0.0), ang_vel_z=(-2.0, 2.0), pos_z=(0.0, 0.1987)
        ),
        initial_phase_time=2.0,
    )

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["left_shoulder_joint", "right_shoulder_joint"],
        scale=0.25,
        use_default_offset=False,
        preserve_order=True,
    )
    wheel_vel = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["left_wheel_joint", "right_wheel_joint"],
        scale=40.0,
        use_default_offset=False,
        preserve_order=True
    )

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""
    @configclass
    class StackCriticCfg(ObsGroup):
        """Observations for critic group to be stacked."""
        joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_joint"])
            },
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel,
            scale=0.15,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_joint", ".*_wheel_joint"])
            },
        )
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel_link, scale=0.25)  # default: -0.15
        base_projected_gravity = ObsTerm(func=mdp.projected_gravity)  # default: -0.05
        actions = ObsTerm(func=mdp.last_action)
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class NoneStackCriticCfg(ObsGroup):
        velocity_commands = ObsTerm(func=mdp.generated_scaled_commands, params={"command_name": "base_velocity", "scale": (2.0, 0.0, 0.25)})
        base_lin_vel_x = ObsTerm(func=mdp.base_lin_vel_x_link, scale=2.0)
        base_lin_vel_y = ObsTerm(func=mdp.base_lin_vel_y_link)
        base_lin_vel_z = ObsTerm(func=mdp.base_lin_vel_z_link, scale=0.25)
        base_pos_z = ObsTerm(func=mdp.base_pos_z_rel_link, params={"sensor_cfg": SceneEntityCfg("base_height_scanner")})
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    @configclass
    class StackPolicyCfg(ObsGroup):
        """Observations for Stack policy group."""
        joint_pos = ObsTerm(
            func=mdp.joint_pos,
            noise=Unoise(n_min=-0.05, n_max=0.05),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_joint"])
            },
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel,
            noise=Unoise(n_min=-1.5, n_max=1.5),
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_shoulder_joint", ".*_wheel_joint"])
            },
            scale=0.15,
        )
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel_link, noise=Unoise(n_min=-0.15, n_max=0.15), scale=0.25)  # default: -0.15
        base_projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))  # default: -0.05
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class NoneStackPolicyCfg(ObsGroup):
        """Observations for None-Stack policy group."""
        velocity_commands = ObsTerm(func=mdp.generated_scaled_commands, params={"command_name": "base_velocity", "scale": (2.0, 0.0, 0.25)})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    stack_policy: StackPolicyCfg = StackPolicyCfg()
    none_stack_policy: NoneStackPolicyCfg = NoneStackPolicyCfg()
    stack_critic: StackCriticCfg = StackCriticCfg()
    none_stack_critic: NoneStackCriticCfg = NoneStackCriticCfg()

@configclass
class EventCfg:
    """Configuration for events."""
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["(?!.*_caster_link)"]),
            "static_friction_range": (0.8, 1.0),
            "dynamic_friction_range": (0.6, 0.8),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )

    randomize_com_positions = EventTerm(
        func=mdp.randomize_com_positions,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "com_distribution_params": (-0.02, 0.02),
            "operation": "add",
        },
    )
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]),
            "mass_distribution_params": (-2.5, 2.5),
            "operation": "add",
        },
    )
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={
            "velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.5, 0.5)},
        },
    )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )
    terrain_out_of_bounds = DoneTerm(
        func=mdp.terrain_out_of_bounds,
        params={"asset_cfg": SceneEntityCfg("robot"), "distance_buffer": 3.0},
        time_out=True,
    )
    shoulder_lower_violation = DoneTerm(
        func=mdp.specific_joint_lower_limit_termination,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "joint_names": ["left_shoulder_joint", "right_shoulder_joint"],
            "threshold": -0.75,
        },
    )
    bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.5})

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)

@configclass
class LocomotionVelocityRoughEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards = MISSING # It will be defined in the task
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material

        # update sensor update periods
        if self. scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

        # check if terrain levels curriculum is enabled - if so, enable curriculum for terrain generator
        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False


@configclass
class LocomotionVelocityFlatEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards = None # It will be defined in the task
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.curriculum.terrain_levels = None

        # update sensor update periods
        if self. scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

        # check if terrain levels curriculum is enabled - if so, enable curriculum for terrain generator
        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False
