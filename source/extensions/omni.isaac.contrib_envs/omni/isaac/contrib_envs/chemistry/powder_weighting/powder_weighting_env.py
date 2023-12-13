import copy
import gym.spaces
import math
import torch
from typing import List
import numpy as np

from pxr import Usd, Gf
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.stage as stage_utils

import omni.isaac.orbit.utils.kit as kit_utils
from omni.isaac.orbit.controllers.differential_inverse_kinematics import DifferentialInverseKinematics,  DifferentialInverseKinematicsCfg
from omni.isaac.orbit.markers import StaticMarker
from omni.isaac.orbit.objects import RigidObject
from omni.isaac.orbit.robots.single_arm import SingleArmManipulator
from omni.isaac.orbit.utils.dict import class_to_dict
from omni.isaac.orbit.utils.math import random_orientation, sample_uniform, scale_transform
from omni.isaac.orbit.utils.mdp import ObservationManager, RewardManager


from omni.isaac.orbit_envs.isaac_env import IsaacEnv, VecEnvIndices, VecEnvObs

from .powder_weighting_cfg import PowderWeightingEnvCfg

from .powder import Powder, PowderCfg

class PowderWeightingEnv(IsaacEnv):
    """ Environment for weighting powder using a single arm manipular """



    def __init__(self, cfg: PowderWeightingEnvCfg, headless: bool = False):
        """ method to create the environment instances and initialize different components"""
        
       
        self.fallen_particles = 0

        self.powder = None

        self.cfg = cfg
        
        # get the robot
        self.robot = SingleArmManipulator(cfg=self.cfg.robot)
        # self.spoon_holder=RigidObject(cfg=self.cfg.spoon_holder)
        # then get the spoon 
        self.spoon = RigidObject(cfg=self.cfg.spoon)
        # platform used to lift the spoon for easier grabing
        # scale 
        self.scale = RigidObject(cfg=self.cfg.scale_ika)
        self.platform=RigidObject(cfg=self.cfg.platform)
        # get beaker 
        # self.beaker = RigidObject(cfg=self.cfg.beaker_500ml)
        self.plate = RigidObject(cfg=self.cfg.plate)
        self.vial = RigidObject(cfg=self.cfg.vial)
        self.objects = [self.spoon, 
                        self.scale,
                        self.vial, 
                        self.plate,
                        # self.beaker,
                        # self.spoon_holder,
                        self.platform
                        ]
        super().__init__(self.cfg, headless=headless, particleSupport=True)
        print("Init happened")
        
        self.prev_step_obs = [None] * len(self.envs_positions)

        self.dt = self.cfg.control.decimation * self.physics_dt  # control-dt
        self.max_episode_length = math.ceil(self.cfg.env.episode_length_s / self.dt)

        self.sim.reset()
        # initialise robot view 
        self.robot.initialize(self.env_ns + "/.*/Robot")
        # is this needed ? 
        self.spoon.initialize(self.env_ns+"/.*/Spoon")
        # self.beaker.initialize(self.env_ns+"/.*/Beaker")
        self.plate.initialize(self.env_ns+"/.*/Plate")
        self.vial.initialize(self.env_ns+"/.*/Vial")
        self.scale.initialize(self.env_ns+"/.*/Scale")
        self.platform.initialize(self.env_ns+"/.*/Platform")
        # self.spoon_holder.initialize(self.env_ns+"/.*/SpoonHolder")
        if self.cfg.control.control_type == "inverse_kinematics":
            ik_control_cfg = DifferentialInverseKinematicsCfg(
                command_type="pose_abs",
                ik_method="dls",
                position_offset=self.robot.cfg.ee_info.pos_offset,
                rotation_offset=self.robot.cfg.ee_info.rot_offset,
            )
            self._ik_controller = DifferentialInverseKinematics(
                self.cfg.control.inverse_kinematics,
                # ik_control_cfg,
                self.robot.count, 
                self.sim.device
            )
            self.num_actions = self._ik_controller.num_actions + 1
        elif self.cfg.control.control_type == "default":
            self.num_actions = self.robot.num_actions

     
        print(self.robot.num_actions)
        self.robot_actions = torch.zeros((self.num_envs, self.robot.num_actions), device=self.device)
        # the traing obeservation manager anc action space need to be set up
        
        #   action_space: The Space object corresponding to valid actions. This should correspond to the action space of a single environment instance.
        #   action a = [incline_action, shake_action] with:
        #         -> incline_action = [-3, 3]
        #         -> shake_action = []
        # self.action_space = gym.spaces.Box(low=np.array([-3.0, 0]), high=np.array([3.0, 1]), shape=(2,))
        
        # !!!! the state machine expects actions in joint space , the paper for
        #  weighting expresses actions differently (see above) Perhaps the translation between the 2 can be 
        #  handled by the agent / instead we just have the joint space for now   
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(self.num_actions,))
        #   observation_space: The Space object corresponding to valid observations. This should correspond to the observation space of a single environment instance.
        #   state = [weight_current, spoon_angle, weight_goal]:
        #           -> weight_current = [-inf, +inf]
        #           -> spoon_angle = [-90, 90]  
        #           -> weight_goal = [5, 15]
        self.observation_space = gym.spaces.Box(low=np.array([-math.inf, -90, 5]), high=np.array([math.inf, 90, 15]), shape=(3,))
        #   reward_range: A tuple corresponding to the min and max possible rewards. A default reward range set to [-inf, +inf] already exists.
        self.reward_range = (-math.inf, 0)

        # self.spwan_particles()

        self.sim.step()
        self.robot.update_buffers(self.dt)
        


    def _design_scene(self)-> List[str]:
        print("-------------->>>>> DESIGN SCENE CALLED")
        kit_utils.create_ground_plane("/World/defaultGroundPlane", z_position=-1.05)
        prim_utils.create_prim(self.template_env_ns + "/Table", usd_path=self.cfg.table.usd_path, translation=(0.2359, 0.0, -0.00378), orientation=(0.71163, 0.0, 0.0, 0.70255))
        self.robot.spawn(self.template_env_ns + "/Robot")
        self.spoon.spawn(self.template_env_ns + "/Spoon")
        self.scale.spawn(self.template_env_ns + "/Scale")
        # self.beaker.spawn(self.template_env_ns+"/Beaker")
        self.plate.spawn(self.template_env_ns + "/Plate")
        self.vial.spawn(self.template_env_ns + "/Vial")
        self.platform.spawn(self.template_env_ns + "/Platform")
        # self.spoon_holder.spawn(self.template_env_ns+"/SpoonHolder")
        # what is this measured in? 
        self.spwan_particles()
        # set visuals for debug
        if self.cfg.viewer.debug_vis and self.enable_render:
            # create marker for viewing end-effector pose
            self._ee_markers = StaticMarker(
                "/Visuals/ee_current",
                self.num_envs,
                usd_path=self.cfg.frame_marker.usd_path,
                scale=self.cfg.frame_marker.scale,
            )
            # create marker for viewing command (if task-space controller is used)
            if self.cfg.control.control_type == "inverse_kinematics":
                self._cmd_markers = StaticMarker(
                    "/Visuals/ik_command",
                    self.num_envs,
                    usd_path=self.cfg.frame_marker.usd_path,
                    scale=self.cfg.frame_marker.scale,
                )
        return ["/World/defaultGroundPlane"]



    def _reset_idx(self, env_ids: VecEnvIndices):
        print("________called_______") 
        # shows which ids will be resetd
        self.reset_buf[env_ids] = 0
        #  holds episode lenght needs to be reseted 
        self.episode_length_buf[env_ids] = 0
        #  set all the assets in the scene to their initial positions
        dof_pos, dof_vel = self.robot.get_default_dof_state(env_ids=env_ids)
        print(dof_pos, dof_vel)
        self.robot.set_dof_state(dof_pos, dof_vel, env_ids=env_ids)
        #  first do all the other objects and then do the powder ? 
        for i in range(len(self.objects)):
            # first we get the initial position of the object 
            root_state = self.objects[i].get_default_root_state(env_ids) 
            # print(  f" -->{root_state} ")
            root_state[:, 0:3] += self.envs_positions[env_ids]
            #  we don't use randomisation now so we just pass it on
            self.objects[i].set_root_state(root_state, env_ids=env_ids)
        if self.powder is not None:
            self.powder.reset()
        


    def _step_impl(self, actions: torch.Tensor):

        # if self.powder is None: 
        #     self.spwan_particles()

        # print(f"---> Partircles stats : {self._count_particles_in_box_environments()}")
        
       
        self.actions = actions.clone().to(device=self.device)
        # set the controller commands from the actins that need to be taken
        # depending on controller type:
        if self.cfg.control.control_type=="inverse_kinematics":
            self._ik_controller.set_command(self.actions[:, :-1])
            #  from task space to joint space 
            self.robot_actions[:, : self.robot.arm_num_dof]=self._ik_controller.compute(
                self.robot.data.ee_state_w[:, 0:3] - self.envs_positions,
                self.robot.data.ee_state_w[:, 3:7],
                self.robot.data.ee_jacobian,
                self.robot.data.arm_dof_pos,
            )
            # we need to adjust for the position offset of the robot
            dof_pos_offset = self.robot.data.actuator_pos_offset
            #  when the ctions get applied to the robot the offsets are added as is. So we don't want
            # them to be included in the action
            self.robot_actions[:, : self.robot.arm_num_dof] -= dof_pos_offset[:, : self.robot.arm_num_dof]
            # we assume last command is gripper action so don't change that
            self.robot_actions[:, -1] = self.actions[:, -1]
            # perform physics stepping
        else:
            self.robot_actions[:]=self.actions
        #  do the stepping, decimation is
        # Number of control action updates @ sim dt per policy dt
        for _ in range(self.cfg.control.decimation):
            # set actions into buffers
            self.robot.apply_action(self.robot_actions)
            # simulate
            self.sim.step(render=self.enable_render)
            # check that simulation is playing
            if self.sim.is_stopped():
                return
        #  compute spoon buffer
        self.spoon.update_buffers(self.dt)
        # self.spoon_holder.update_buffers(self.dt)
        # needed to make the robot move 
        self.robot.update_buffers(self.dt)
        #  post step executions
        self._check_termination()

        # -- update USD visualization
        if self.cfg.viewer.debug_vis and self.enable_render:
            self._debug_vis()



    def _get_observations(self) -> VecEnvObs:
        pass 




    # # # # # # # # # # # #
    #  Step implementation helper funcitons
    # # # # # # # # # # # #

    def _debug_vis(self):
        """Visualize the environment in debug mode."""
        # apply to instance manager
        # -- end-effector
        self._ee_markers.set_world_poses(self.robot.data.ee_state_w[:, 0:3], self.robot.data.ee_state_w[:, 3:7])
        # -- task-space commands
        if self.cfg.control.control_type == "inverse_kinematics":
            # convert to world frame
            ee_positions = self._ik_controller.desired_ee_pos + self.envs_positions
            ee_orientations = self._ik_controller.desired_ee_rot
            # set poses
            self._cmd_markers.set_world_poses(ee_positions, ee_orientations)



    def _check_termination(self):
        if self.cfg.terminations.episode_timeout:
            self.reset_buf = torch.where(self.episode_length_buf >= self.max_episode_length, 1, self.reset_buf) 

    # # # # # # # # # # # #
    #  Scene helper functions
    # # # # # # # # # # # #


    def spwan_particles(self):
        """
            This function spawns the needed particles in each environment. 
            Should be executed outside of design_scene so that cloner is not involved. 
            Creates finer control over the particles
        """
        powderCfg = PowderCfg(
                particleDiameter=0.001,
                friction = 0.7, 
                adhesion = 0.5,  
                mass =   0.0000000033, 
            )
        stage=stage_utils.get_current_stage()

        powder = Powder(stage, powderCfg, path_prefix=self.template_env_ns)
        #  vial powder
        # powder.create_powder_cylinder(Gf.Vec3f(0.54, 0.01, 0.15),0.002, 0.002)
        self.powder = powder
        #  beaker/plate powder 
        self.powder.create_powder_cylinder(Gf.Vec3f(0.53, 0.25, 0.06), 0.033, 0.037)


    # # # # # # # # # # # #
    #  Observation Manager helper functions
    # # # # # # # # # # # #
    
    def _count_particles_in_vial(self):
        # VIAL_center_top = 20 / 1000
        VIAL_center_top = 27/1000
        VIAL_center_bottom = 27/1000
        # VIAL_center_bottom = 30 / 1000
        # VIAL_RAD = 8 / 1000
        VIAL_RAD = 12.5/1000
        positions = self.powder.get_positions() 
        real_center_top = VIAL_center_top * self.cfg.vial.meta_info.scale[0]
        real_center_bottom = VIAL_center_bottom * self.cfg.vial.meta_info.scale[0]
        real_rad = VIAL_RAD * self.cfg.vial.meta_info.scale[0]

        

        vial_pose = self.vial.objects.get_world_poses()
        vial_position = vial_pose[0][0].numpy()  # Convert tensor to NumPy array
        
        z_min = vial_position[2] - real_center_bottom
        z_max = vial_position[2] + real_center_top

        # Extract x, y, z coordinates of positions
        positions_xyz = np.array(positions)[0]
        

        # Check positions within height range (z-axis)
        height_mask = np.logical_and(positions_xyz[:, 2] >= z_min, positions_xyz[:, 2] <= z_max)

        # Extract x, y coordinates of positions for distance calculations
        positions_xy = positions_xyz[:, :2]
        
        vial_xy = vial_position[:2]
        
        # Calculate squared distances
        dist_squared = np.sum((positions_xy - vial_xy) ** 2, axis=1)

        # Check distances within the radius
        rad_mask = dist_squared <= real_rad ** 2

        # Count particles satisfying height and distance conditions
        counter = np.sum(height_mask & rad_mask)

        return counter

    def _count_particles_in_box_environments(self):
        particle_stats = []
        for i in range(len(self.envs_positions)):
            particle_stats.append(self._count_particles_in_vial_box(i))
        return particle_stats

    
    def _count_particles_in_vial_box(self, env_id):
        
        
        l = 30/1000
        L = 28/1000
        z_offset = 17 / 1000
        
        positions = self.powder.get_positions(env_id)
        real_l = l * self.cfg.vial.meta_info.scale[0]
        real_L = L * self.cfg.vial.meta_info.scale[0]
        real_z_offset = z_offset * self.cfg.vial.meta_info.scale[0]

        vial_pose = self.vial.objects.get_world_poses() 
        vial_position = vial_pose[0][env_id].numpy()   # Convert tensor to NumPy array
        z_min = vial_position[2] +  real_z_offset
        z_max = vial_position[2] + real_L + real_z_offset

        y_min = vial_position[1] - real_l/2
        y_max = vial_position[1] + real_l/2

        x_min = vial_position[0] - real_l/2
        x_max = vial_position[0] + real_l/2

        # Extract x, y, z coordinates of positions
        # print(np.array(self.envs_positions[env_id]))
        positions_xyz = np.array(positions)[0]  + np.array(self.envs_positions[env_id])
        # print(np.shape(positions_xyz))

        # Create boolean masks for each axis separately
        mask_x = (positions_xyz[:, 0] >= x_min) & (positions_xyz[:, 0] <= x_max)
        mask_y = (positions_xyz[:, 1] >= y_min) & (positions_xyz[:, 1] <= y_max)
        mask_z = (positions_xyz[:, 2] >= z_min) & (positions_xyz[:, 2] <= z_max)
        mask_z_under = (positions_xyz[:, 2] < z_min)
        # Combine all masks using logical AND
        final_condition = np.logical_and.reduce([mask_x, mask_y, mask_z])

        #  a particle that was previously in should be in at the next step too 
        if self.prev_step_obs[env_id] is not None:
            # Take all the truths ever
            self.prev_step_obs[env_id] = self.prev_step_obs[env_id] | final_condition
        else:
            # if first time save all particles that were in at s_i
            self.prev_step_obs[env_id] = final_condition.copy()

        # Get the indices where the conditions are met
        counter = np.count_nonzero(final_condition)
        fallen = np.count_nonzero(self.prev_step_obs[env_id] & ~final_condition)

        return counter, fallen


        
        
