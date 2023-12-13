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

from powder import Powder , PowderCfg
import omni.isaac.core.utils.stage as stage_utils
from pxr import Usd, Gf, Sdf, UsdPhysics, UsdGeom
from omni.physx.scripts import physicsUtils, particleUtils
import omni.usd

def main():
    sim = SimulationContext(physics_dt=0.01, rendering_dt=0.01, backend="torch", set_defaults=False)
    stage = stage_utils.get_current_stage()
    # Set main camera
    set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

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
        particleDiameter=0.03, 
        friction = 0.7,
        adhesion = 0.8
    )
    powder = Powder(stage, powderCfg)

    # spoon import 
    # spoon_usd_path = f"{assets.ASSETS_DATA_DIR}/objects/Spoons/spoon2.usd"
    # prim_utils.create_prim("/World/Spoon", usd_path=spoon_usd_path, translation=(0.0, 0.0, -1.0), orientation=(0.7933533, 0.6087614, 0, 0))
    

    # for now the particles can be sampled onely when created from this 
    cube_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, "Cone", True))
    cube_resolution = ( 2  # resolution can be low because we'll sample the surface / volume only irrespective of the vertex count
    )
    omni.kit.commands.execute(
        "CreateMeshPrimWithDefaultXform", prim_type="Cone", u_patches=cube_resolution, v_patches=cube_resolution
    )
    
    
    cube_mesh = UsdGeom.Mesh.Get(stage, cube_mesh_path)
    physicsUtils.set_or_add_translate_op(cube_mesh, Gf.Vec3f(0.0, 0.0, 0))

    
    powder.powder_from_mesh(cube_mesh)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Simulate physics

    while simulation_app.is_running():
        # If simulation is stopped, then exit.
        if sim.is_stopped():
            break

        # If simulation is paused, then skip.
        if not sim.is_playing():
            sim.step(render=not args_cli.headless)
            continue

        # perform step
        sim.step()

if __name__ == "__main__":
    main()
    simulation_app.close()