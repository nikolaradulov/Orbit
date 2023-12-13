# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
# from omni.isaac.assets import ASSETS_DATA_DIR

from omni.isaac.orbit.controllers.differential_inverse_kinematics import DifferentialInverseKinematicsCfg
from omni.isaac.orbit.objects import RigidObjectCfg
from omni.isaac.orbit.robots.config.franka_8dof_robotiq import FRANKA_PANDA_ARM_8DOF_WITH_ROBOTIQ_85_GRIPPER_CFG
from omni.isaac.orbit.robots.single_arm import SingleArmManipulatorCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.assets import ASSETS_DATA_DIR

from omni.isaac.orbit_envs.isaac_env_cfg import EnvCfg, IsaacEnvCfg, PhysxCfg, SimCfg, ViewerCfg
import os
##
# Scene settings
##


@configclass
class TableCfg:
    """Properties for the table."""

    # note: we use instanceable asset since it consumes less memory
    # usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
    usd_path = os.path.join(ASSETS_DATA_DIR, "objects", "table", "table_instanceable.usda")


@configclass
class ManipulationBeaker500mLCfg(RigidObjectCfg):
    """Properties for the object to manipulate in the scene."""

    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ASSETS_DATA_DIR}/objects/Beakers/beaker-500mL_inst.usda",
        scale=(0.4, 0.4, 0.4),
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.6, 0.1, 0.05), rot=(1.0, 0.0, 0.0, 0.0), lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)
    )
    rigid_props = RigidObjectCfg.RigidBodyPropertiesCfg(
        max_angular_velocity=1000.0,
        max_linear_velocity=1000.0,
        max_depenetration_velocity=10.0,
        disable_gravity=False,
    )
    physics_material = RigidObjectCfg.PhysicsMaterialCfg(
        static_friction=0.5, dynamic_friction=0.5, restitution=0.0, prim_path="/World/Materials/cubeMaterial"
    )


@configclass
class ManipulationScaleIKACfg(RigidObjectCfg):
    """Properties for the object to manipulate in the scene."""

    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ASSETS_DATA_DIR}/objects/LabTools/scale-IKA_inst.usda",
        scale=(1.2, 1.2, 1.2),
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.4, 0.3, 0.1), rot=(0.7071068, 0.7071068, 0.0, 0.0), lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)
    )
    rigid_props = RigidObjectCfg.RigidBodyPropertiesCfg(
        max_angular_velocity=1000.0,
        max_linear_velocity=1000.0,
        max_depenetration_velocity=10.0,
        disable_gravity=False,
    )
    physics_material = RigidObjectCfg.PhysicsMaterialCfg(
        static_friction=0.5, dynamic_friction=0.5, restitution=0.0, prim_path="/World/Materials/cubeMaterial"
    )


@configclass
class GoalMarkerCfg:
    """Properties for visualization marker."""

    # usd file to import
    usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd"
    # scale of the asset at import
    scale = [0.05, 0.05, 0.05]  # x,y,z


@configclass
class FrameMarkerCfg:
    """Properties for visualization marker."""

    # usd file to import
    usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd"
    # scale of the asset at import
    scale = [0.1, 0.1, 0.1]  # x,y,z


##
# MDP settings
##


@configclass
class RandomizationCfg:
    """Randomization of scene at reset."""

    @configclass
    class ObjectInitialPoseCfg:
        """Randomization of object initial pose."""

        # category
        position_cat: str = "default"  # randomize position: "default", "uniform"
        orientation_cat: str = "default"  # randomize position: "default", "uniform"
        # randomize position
        position_uniform_min = [0.25, -0.25, 0.25]  # position (x,y,z)
        position_uniform_max = [0.5, 0.25, 0.5]  # position (x,y,z)

    @configclass
    class ObjectDesiredPoseCfg:
        """Randomization of object desired pose."""

        # category
        position_cat: str = "default"  # randomize position: "default", "uniform"
        orientation_cat: str = "default"  # randomize position: "default", "uniform"
        # randomize position
        position_default = [0.5, 0.0, 0.5]  # position default (x,y,z)
        position_uniform_min = [0.25, -0.25, 0.25]  # position (x,y,z)
        position_uniform_max = [0.5, 0.25, 0.5]  # position (x,y,z)
        # randomize orientation
        orientation_default = [1.0, 0.0, 0.0, 0.0]  # orientation default

    # initialize
    object_initial_pose: ObjectInitialPoseCfg = ObjectInitialPoseCfg()
    object_desired_pose: ObjectDesiredPoseCfg = ObjectDesiredPoseCfg()


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg:
        """Observations for policy group."""

        # global group settings
        enable_corruption: bool = True
        # observation terms
        arm_dof_pos_scaled = {"scale": 1.0, "noise": {"name": "uniform", "min": -0.01, "max": 0.01}}
        tool_dof_pos_scaled = {"scale": 1.0}
        arm_dof_vel = {"scale": 0.5, "noise": {"name": "uniform", "min": -0.1, "max": 0.1}}
        tool_positions = {}
        object_positions = {}
        object_desired_positions = {}
        actions = {}

    # global observation settings
    return_dict_obs_in_group = False
    """Whether to return observations as dictionary or flattened vector within groups."""
    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # robot-centric
    reaching_object_position_l2 = {"weight": 0.0}
    reaching_object_position_exp = {"weight": 2.5, "sigma": 0.25}
    penalizing_robot_dof_velocity_l2 = {"weight": 1e-4}
    penalizing_robot_dof_acceleration_l2 = {"weight": 1e-7}
    penalizing_action_rate_l2 = {"weight": 1e-2}
    # object-centric
    tracking_object_position_exp = {"weight": 2.5, "sigma": 0.5}
    lifting_object_success = {"weight": 0.0, "threshold": 1e-3}


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    episode_timeout = True  # reset when episode length ended
    object_falling = True  # reset when object falls off the table
    is_success = False  # reset when object is lifted


@configclass
class ControlCfg:
    """Processing of MDP actions."""

    # action space
    control_type = "default"  # "default", "inverse_kinematics"
    # decimation: Number of control action updates @ sim dt per policy dt
    decimation = 2

    # configuration loaded when control_type == "inverse_kinematics"
    inverse_kinematics: DifferentialInverseKinematicsCfg = DifferentialInverseKinematicsCfg(
        command_type="pose_rel",
        ik_method="dls",
        position_command_scale=(0.1, 0.1, 0.1),
        rotation_command_scale=(0.1, 0.1, 0.1),
    )


##
# Environment configuration
##


@configclass
class ChemistryV1EnvCfg(IsaacEnvCfg):
    # General Settings
    env: EnvCfg = EnvCfg(num_envs=1024, env_spacing=2.5, episode_length_s=15.0)
    viewer: ViewerCfg = ViewerCfg(debug_vis=True, eye=(7.5, 7.5, 7.5), lookat=(0.0, 0.0, 0.0))
    # Physics settings
    sim: SimCfg = SimCfg(
        dt=1.0 / 100.0,
        substeps=1,
        physx=PhysxCfg(
            gpu_found_lost_aggregate_pairs_capacity=512 * 1024,
            gpu_total_aggregate_pairs_capacity=6 * 1024,
        ),
    )

    # Scene Settings
    # -- robot
    # robot: SingleArmManipulatorCfg = FRANKA_PANDA_ARM_WITH_PANDA_HAND_CFG
    robot: SingleArmManipulatorCfg = FRANKA_PANDA_ARM_8DOF_WITH_ROBOTIQ_85_GRIPPER_CFG
    robot.data_info.enable_jacobian = True
    robot.rigid_props.disable_gravity = True  # this option will disable the gravity terms on robot
    # robot.data_info.enable_gravity  # this option will buffer the gravity terms from the robot
    # -- object
    beaker_500ml: ManipulationBeaker500mLCfg = ManipulationBeaker500mLCfg()
    scale_ika: ManipulationScaleIKACfg = ManipulationScaleIKACfg()

    # -- table
    table: TableCfg = TableCfg()
    # -- visualization marker
    goal_marker: GoalMarkerCfg = GoalMarkerCfg()
    frame_marker: FrameMarkerCfg = FrameMarkerCfg()

    # MDP settings
    randomization: RandomizationCfg = RandomizationCfg()
    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Controller settings
    control: ControlCfg = ControlCfg()
