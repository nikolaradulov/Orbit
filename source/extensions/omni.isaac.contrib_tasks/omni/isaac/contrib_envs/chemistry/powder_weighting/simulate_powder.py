import argparse

from omni.isaac.kit import SimulationApp

# add argparse arguments
parser = argparse.ArgumentParser("Welcome to Orbit: Omniverse Robotics Environments!")
parser.add_argument("--headless", action="store_true", default=False, help="Force display off at all times.")
parser.add_argument("--robot", type=str, default="franka_panda", help="Name of the robot.")
args_cli = parser.parse_args()

# launch omniverse app
config = {"headless": args_cli.headless}
simulation_app = SimulationApp(config)


"""Rest everything follows."""


import torch
import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.viewports import set_camera_view

import omni.isaac.orbit.utils.kit as kit_utils
from omni.isaac.orbit.robots.config.franka import FRANKA_PANDA_ARM_WITH_PANDA_HAND_CFG
from omni.isaac.orbit.robots.config.universal_robots import UR10_CFG
from omni.isaac.orbit.robots.single_arm import SingleArmManipulator
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR
import omni.isaac.assets as assets
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid

from powder import Powder, PowderCfg
import omni.isaac.core.utils.stage as stage_utils
from pxr import Usd, Gf, Sdf, UsdPhysics, UsdGeom
from omni.physx.scripts import physicsUtils, particleUtils
import omni.usd

import time

fallen_particles = 0
powder = None 
vial = None

def main():

    start_time = time.time()
    global vial
    sim = SimulationContext(physics_dt=1.0/200.0,
                            rendering_dt=1.0/200.0, 
                            backend="torch", 
                            set_defaults=False
                            )
    stage = stage_utils.get_current_stage()
    # Set main camera
    set_camera_view([1.0, 1.0, 0.5], [0.0, 0.0, -1.0])

    # Spawn things into stage
    # Ground-plane
    kit_utils.create_ground_plane("/World/defaultGroundPlane", z_position=-1.05)
    # Lights-1
    prim_utils.create_prim(
        "/World/Light/GreySphere",
        "SphereLight",
        translation=(4.5, 3.5, 10.0),
        attributes={"radius": 2.5, "intensity": 600.0, "color": (0.75, 0.75, 0.75)},
    )
    powderCfg = PowderCfg(
        particleDiameter=0.001, 
        friction = 0.7,
        adhesion = 0.8,
        mass =  0.0000000033 #this is in kg 
    )
    global powder 
    powder = Powder(stage, powderCfg)
    # powder.create_powder_ball(Gf.Vec3f(0.49, 0.0, -1.0), 0.005)
    powder.create_powder_cylinder(Gf.Vec3f(0.0, 0.0, -0.7), 0.015, 0.014)
    # spoon import 
    beaker_usd_path = f"{assets.ASSETS_DATA_DIR}/objects/PowderRecipient/cubeV4.usda"
    spoon_usd_path = f"{assets.ASSETS_DATA_DIR}/objects/Spoons/Big_Spoon.usd"
    # orientation=(0.7933533, 0.6087614, 0, 0)
    # scale 10 for beaker
    # orientation=(0.7132504 , 0.7009093, 0, 0),  
    # prim_utils.create_prim("/World/Spoon", usd_path=spoon_usd_path, translation=(0.5, 0.0, -1.0), scale=(0.001, 0.001, 0.001))
    # beaker
    vial = prim_utils.create_prim("/World/Beaker", usd_path=beaker_usd_path, translation=(0, 0.0, -1.0), scale=(1, 1, 1), orientation=(1, 0.0, 0.0, 0.0))
    
    # powder.powder_from_mesh(cube_mesh)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Simulate physics
    elapsed_time = 0
    while simulation_app.is_running() :
        # If simulation is stopped, then exit.
        if sim.is_stopped():
            break

        # If simulation is paused, then skip.
        if not sim.is_playing():
            sim.step(render=not args_cli.headless)
            continue

        print(_count_particles_in_vial_box())

        # perform step
        sim.step()

        elapsed_time  = time.time() - start_time
        if(elapsed_time>5):
            sim.reset()
            elapsed_time = 0
            start_time = time.time()

prev_step_obs = None
fallen_particles = 0

def _count_particles_in_vial_box():
        
        global prev_step_obs
        global fallen_particles
        l = 40/1000
        L = 30/1000
        positions = powder.get_positions()
        real_l = l * 1
        real_L = L * 1
        z_offset = 17 / 1000 * 1
        matrix: Gf.Matrix4d = omni.usd.get_world_transform_matrix(vial)
        vial_pose: Gf.Vec3d = matrix.ExtractTranslation()
        # print(vial_pose)
        vial_position = np.array(vial_pose)  # Convert tensor to NumPy array
        # print(vial_position)
        z_min = vial_position[2] + z_offset
        z_max = vial_position[2] + real_L + z_offset

        y_min = vial_position[1] - real_l/2
        y_max = vial_position[1] + real_l/2

        x_min = vial_position[0] - real_l/2
        x_max = vial_position[0] + real_l/2

        # Extract x, y, z coordinates of positions
        positions_xyz = np.array(positions)[0]
        

        # Create boolean masks for each axis separately
        mask_x = (positions_xyz[:, 0] >= x_min) & (positions_xyz[:, 0] <= x_max)
        mask_y = (positions_xyz[:, 1] >= y_min) & (positions_xyz[:, 1] <= y_max)
        mask_z = (positions_xyz[:, 2] >= z_min) & (positions_xyz[:, 2] <= z_max)

        # Combine all masks using logical AND
        final_condition = np.logical_and.reduce([mask_x, mask_y, mask_z])
        felled_particles = 0
        # #  a particle that was previously in should be in at the next step too 
        # if prev_step_obs is not None:
        #     felled_particles = np.count_nonzero(prev_step_obs & ~final_condition)
        #     fallen_particles += felled_particles
        # svae the current particles for latter comparison

        prev_step_obs = final_condition.copy()

        # Get the indices where the conditions are met
        counter = np.count_nonzero(final_condition)
        fallen_particles = np.count_nonzero(~final_condition)

        return counter, fallen_particles



if __name__ == "__main__":
    main()
    simulation_app.close()