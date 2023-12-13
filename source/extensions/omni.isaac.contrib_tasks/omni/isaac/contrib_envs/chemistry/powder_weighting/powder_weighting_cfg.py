from omni.isaac.orbit.controllers.differential_inverse_kinematics import DifferentialInverseKinematicsCfg
from omni.isaac.orbit.objects import RigidObjectCfg
from omni.isaac.orbit.robots.config.franka_8dof_robotiq import FRANKA_PANDA_ARM_8DOF_WITH_ROBOTIQ_85_GRIPPER_CFG
from omni.isaac.orbit.robots.single_arm import SingleArmManipulatorCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.assets import ASSETS_DATA_DIR
from omni.isaac.orbit.robots.config.franka import FRANKA_PANDA_ARM_WITH_PANDA_HAND_CFG


from omni.isaac.orbit_envs.isaac_env_cfg import EnvCfg, IsaacEnvCfg, PhysxCfg, SimCfg, ViewerCfg
import os

@configclass
class TableCfg:
    """Properties for the table."""

    # note: we use instanceable asset since it consumes less memory
    # usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
    usd_path = os.path.join(ASSETS_DATA_DIR, "objects", "tables", "vention_table", "vention_table_inst.usda")

@configclass
class ManipulationScaleIKACfg(RigidObjectCfg):
    """Properties for the scale to manipulate in the scene."""

    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ASSETS_DATA_DIR}/objects/LabTools/scale-IKA_inst.usda",
        scale=(1.6, 1.6, 1.6),
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.55, 0.0, 0.0), rot=(0.7071068, 0.7071068, 0.0, 0.0), lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)
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
class SpoonHolderCfg(RigidObjectCfg):
    meta_info=RigidObjectCfg.MetaInfoCfg(
        usd_path = f"{ASSETS_DATA_DIR}/objects/Spoons/SpoonHolder.usd",
        scale=(0.00095, 0.00095, 0.00095)
    )

    init_state = RigidObjectCfg.InitialStateCfg(
        # pos = (0.52972, -0.19555, 0.00133),
        pos = (0.53, -0.19514, -0.00127),
        # rot= (0.00386, -0.00009, 0.00009, 0.99999),
        rot = ( 0.0, 0.0, 0.0, 1.0), 
        lin_vel = (0.0, 0.0, 0.0),
        ang_vel = (0.0, 0.0 ,0.0)
    )
    rigid_props = RigidObjectCfg.RigidBodyPropertiesCfg(
        max_angular_velocity=1000.0,
        max_linear_velocity=1000.0,
        max_depenetration_velocity=10.0,
        disable_gravity=False,
    )

@configclass 
class SpoonPlatformCfg(RigidObjectCfg):
    meta_info=RigidObjectCfg.MetaInfoCfg(
        usd_path = f"{ASSETS_DATA_DIR}/objects/Spoons/platform-deeper.usda",
        scale=(1, 1, 1)
    )

    init_state = RigidObjectCfg.InitialStateCfg(
        pos = (0.55,  -0.3, 0.02),
        rot = ( 0.0, 0.0, 0.0, 1.0), 
        lin_vel = (0.0, 0.0, 0.0),
        ang_vel = (0.0, 0.0 ,0.0)
    )
    rigid_props = RigidObjectCfg.RigidBodyPropertiesCfg(
        max_angular_velocity=1000.0,
        max_linear_velocity=1000.0,
        max_depenetration_velocity=10.0,
        disable_gravity=False,
    )

@configclass
class SpoonCfg(RigidObjectCfg):
    #  if the spoon has to suffer randomisation of attributes then how the fuck 
    #  do I make it
    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path = f"{ASSETS_DATA_DIR}/objects/Spoons/large-base-spoon.usda",
        scale = (0.0009, 0.0009, 0.0009)
    )

    # this will need to be in the manipulator of the robot
    # for now just place it on the table 
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.6, -0.31 , 0.05),
        rot=(0.7071068, 0.0, 0.0, -0.7071068),
        # spoon vertical pos
        # pos = (0.52488, -0.22529, 0.01541),
        # rot = (-0.51466, 0.47153, -0.48202, 0.52957),
        lin_vel = (0.0, 0.0, 0.0),
        ang_vel = (0.0, 0.0 ,0.0)
    )
    # rigit props probably also needed 
    rigid_props = RigidObjectCfg.RigidBodyPropertiesCfg(
        max_angular_velocity=1000.0,
        max_linear_velocity=1000.0,
        max_depenetration_velocity=10.0,
        disable_gravity=False,
    )
    # physics material details will also need to be set
    # in the paper the spoon friction coefficient is randomly set 


# vial config
@configclass
class VialCfg(RigidObjectCfg):
    """Properties for the object to manipulate in the scene."""

    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ASSETS_DATA_DIR}/objects/vials/vial_20ml/vial-convex.usda",
        #  for the original vial use 1.2
        scale = (1.0, 1.0, 1.0)
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.54, 0.01, 0.13),
        rot = (0.70711, 0.70711, 0.0, 0.0),
        # rot = (0.45056, 0.00624, -0.00961, 0.89267),
        lin_vel=(0.0, 0.0, 0.0), 
        ang_vel=(0.0, 0.0, 0.0)
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

# box config
@configclass
class BoxCfg(RigidObjectCfg):
    """Properties for the object to manipulate in the scene."""

    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ASSETS_DATA_DIR}/objects/PowderRecipient/cubeV4.usda",
        #  for the original vial use 1.2
        scale = (1.0, 1.0, 1.0)
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.54, 0.01, 0.09),
        rot = (1, 0.0, 0.0, 0.0),
        # rot = (0.45056, 0.00624, -0.00961, 0.89267),
        lin_vel=(0.0, 0.0, 0.0), 
        ang_vel=(0.0, 0.0, 0.0)
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
class PlateCfg(RigidObjectCfg):
    """Properties for the object to manipulate in the scene."""

    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ASSETS_DATA_DIR}/objects/PowderRecipient/plate-inst-thick.usda",
        scale=(1, 1, 1),
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.53, 0.25, 0.02),
        rot=(1.0, 0.0, 0.0, 0.0),
        lin_vel=(0.0, 0.0, 0.0), 
        ang_vel=(0.0, 0.0, 0.0)
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
class ManipulationBeaker500mLCfg(RigidObjectCfg):
    meta_info = RigidObjectCfg.MetaInfoCfg(
        usd_path=f"{ASSETS_DATA_DIR}/objects/Beakers/beaker-500mL.usda",
        #  0.8 scale ~ 500ML -> 0.16 ~ 100ML
        scale=(0.65, 0.65, 0.65),
    )
    init_state = RigidObjectCfg.InitialStateCfg(
        pos=(0.53, 0.25, 0.01),
        # pos=(0.535, 0.01, 0.1), 
        # rot=(0.66483, 0.67978, -0.1387, -0.23653), 
        #  rotation 500mL mesh
        rot = (0.31017, 0.31871, 0.62638, 0.6402),
        # rot=(0.0, 0.0, 0.0, 0.0),
        lin_vel=(0.0, 0.0, 0.0), 
        ang_vel=(0.0, 0.0, 0.0)
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

@configclass
class FrameMarkerCfg:
    """Properties for visualization marker."""

    # usd file to import
    usd_path = f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd"
    # scale of the asset at import
    scale = [0.05, 0.05, 0.05]  # x,y,z

@configclass
class PowderWeightingEnvCfg(IsaacEnvCfg):
    # General Settings 
    # need to figure the episode length , it's probably in seconds. Need to check out with 
    # the paper
    env: EnvCfg = EnvCfg(num_envs=32, env_spacing=3, episode_length_s=15.0)
    
    viewer: ViewerCfg = ViewerCfg(debug_vis=True, eye=(8.0, 8.0, 8.0), lookat=(0.0, 0.0, 0.0))   
    sim: SimCfg = SimCfg(
        dt=1.0 / 200.0,
        substeps=1,
        physx=PhysxCfg(
            use_gpu=True,
        ),
        use_flatcache=True,
        use_gpu_pipeline=True,
        device="cpu",
        disable_contact_processing=True,
        enable_scene_query_support=True,
    )
    

    # robot: SingleArmManipulatorCfg = FRANKA_PANDA_ARM_WITH_PANDA_HAND_CFG
    robot: SingleArmManipulatorCfg = FRANKA_PANDA_ARM_8DOF_WITH_ROBOTIQ_85_GRIPPER_CFG
    # change the robot control settings for smoother motion
    # robot.actuator_groups["panda_shoulder"].control_cfg.damping = {".*": 80}
    # robot.actuator_groups["panda_forearm"].control_cfg.damping = {".*": 80}
    # # change gripper settings for better grasping
    # robot.actuator_groups["panda_hand"].control_cfg.command_types = ["p_abs"]
    # robot.actuator_groups["panda_hand"].control_cfg.stiffness = {".*": 1.3e3}
    robot.data_info.enable_jacobian = True
    robot.rigid_props.disable_gravity = True  # this option will disable the gravity terms on robot
    # robot.data_info.enable_gravity  # this option will buffer the gravity terms from the robot
    # scale for the robot
    scale_ika: ManipulationScaleIKACfg = ManipulationScaleIKACfg()
    # -- table
    table: TableCfg = TableCfg()
    # spoon 
    spoon: SpoonCfg=SpoonCfg()
    # vial 
    # vial: VialCfg=VialCfg()
    # box 
    vial: BoxCfg=BoxCfg()
    #  spoon holder
    spoon_holder: SpoonHolderCfg=SpoonHolderCfg()
    # beaker   
    beaker_500ml: ManipulationBeaker500mLCfg = ManipulationBeaker500mLCfg()
    plate: PlateCfg=PlateCfg()

    platform: SpoonPlatformCfg=SpoonPlatformCfg()
    # ???
    control: ControlCfg = ControlCfg()
    terminations: TerminationsCfg = TerminationsCfg()

        # -- visualization marker
    frame_marker: FrameMarkerCfg = FrameMarkerCfg()