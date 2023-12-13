import math
from pxr import UsdLux, UsdGeom, Sdf, Gf, Vt, UsdPhysics, PhysxSchema
from omni.physx.scripts import utils
from omni.physx.scripts import physicsUtils, particleUtils
import omni.physx.bindings._physx as physx_settings_bindings
import omni.timeline
import numpy as np
from pxr import UsdGeom, Sdf, Gf, PhysxSchema, UsdLux, UsdPhysics, UsdShade
import omni.isaac.core.materials as materials
import omni.isaac.core.prims as prims
from dataclasses import dataclass
import omni.isaac.core.utils.prims as prim_utils

@dataclass
class PowderCfg():
    particleDiameter: float = 0.0045
    # used for old calculations of contact
    # particleSpacing: float = 0.005
    friction: float = None 
    particle_friction_scale: float = None 
    damping: float = None 
    viscosity: float = None 
    vorticity_confinement: float = None 
    surface_tension: float = None 
    cohesion: float = None 
    adhesion: float = None 
    particle_adhesion_scale: float = None 
    adhesion_offset_scale: float = None 
    gravity_scale: float = None
    lift: float = None 
    drag: float = None
    fluid: bool = False
    mass: float = 0.001
    density: float = 0.0


class Powder():
        
    def __init__(self, stage, cfg, path_prefix: str = ""):
        """
            Sets the particle system , the pbd material and the 
            particle objects. All these are needed for the particle 
            simulation: 
            https://docs.omniverse.nvidia.com/extensions/latest/ext_physics/physics-particles.html
        """
        self.cfg= cfg
        self.path_prefix = path_prefix
        self._rng_seed = 42
        self._rng = np.random.default_rng(self._rng_seed)
        self.stage=stage
        # Physics scene
        self.scenePath = Sdf.Path("/physicsScene")
        self.scene = UsdPhysics.Scene.Define(stage, self.scenePath)
        self.scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self.scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Particle System
        self.particleSystemPath = Sdf.Path(self.path_prefix+"/particleSystem")
       
        
        self.solidRestOffset = self.cfg.particleDiameter/2
        self.restOffset = self.solidRestOffset
        self.fluidRestOffset = 0.6 * self.solidRestOffset
        # no idea why this only works with the max condition
        # self.particleContactOffset =  max(self.solidRestOffset, self.solidRestOffset+0.005)
        self.particleContactOffset = self.solidRestOffset *1.5
        self.contactOffset=self.particleContactOffset
        
        self.particle_system = prims.ParticleSystem(self.path_prefix+"/particleSystem", simulation_owner='/physicsScene', 
            particle_contact_offset=self.particleContactOffset,
            contact_offset=self.contactOffset,
            rest_offset=self.restOffset,
            solid_rest_offset=self.solidRestOffset,
            fluid_rest_offset=self.fluidRestOffset,
            )
        # Create a pbd particle material and set it on the particle system
        particle_material = materials.ParticleMaterial(self.path_prefix+"/particleMaterial", 
            friction=self.cfg.friction,
            particle_friction_scale=self.cfg.particle_friction_scale ,
            damping=self.cfg.damping,
            viscosity=self.cfg.viscosity, 
            vorticity_confinement=self.cfg.vorticity_confinement,
            surface_tension=self.cfg.surface_tension,
            cohesion=self.cfg.cohesion, 
            adhesion=self.cfg.adhesion, 
            particle_adhesion_scale=self.cfg.particle_adhesion_scale ,
            adhesion_offset_scale=self.cfg.adhesion_offset_scale,
            gravity_scale=self.cfg.gravity_scale,
            lift=self.cfg.lift ,
            drag=self.cfg.drag)
        self.particle_system.apply_particle_material(particle_material)
        # retains all the particle prims currently in scope
        self.particle_prims = []
        self.id_count = 0

    def create_powder_ball(self, position, radius, name=None):
        if name is None:
            name = "/Particles"+str(self.id_count)
        self.particle_prims.append(("ball", name, position, 0, radius))
        self.id_count+=1
        particleSpacing = self.solidRestOffset*2
        points = []
        dim = math.ceil(2 * radius / particleSpacing)
        for i in range(dim):
            for j in range(dim):
                for k in range(dim):
                    x = i * particleSpacing - radius 
                    y = j * particleSpacing - radius 
                    z = k * particleSpacing - radius 

                    d2 = x * x + y * y + z * z
                    if d2 < radius * radius:
                        points.append(Gf.Vec3f(x, y, z))

        self.__create_point_prim(position, points, name)

    def powder_from_mesh(self, object): 
        particlePointsPath = Sdf.Path("/sampledParticles")
        points = UsdGeom.Points.Define(self.stage, particlePointsPath)
        particle_set_api = PhysxSchema.PhysxParticleSetAPI.Apply(points.GetPrim())
        particle_set_api.CreateParticleSystemRel().SetTargets([Sdf.Path('/particleSystem')])
        particle_sampler_distance = 2.0 * self.solidRestOffset
        sampling_api = PhysxSchema.PhysxParticleSamplingAPI.Apply(object.GetPrim())
        sampling_api.CreateParticlesRel().AddTarget(particlePointsPath)
        sampling_api.CreateSamplingDistanceAttr().Set(particle_sampler_distance)
        sampling_api.CreateMaxSamplesAttr().Set(5e5)
        sampling_api.CreateVolumeAttr().Set(True)

    def reset(self):
        #  pop all the particle prims
        prims_to_empty = len(self.particle_prims)
        counter = 0
        while(len(self.particle_prims)>0 and counter <=prims_to_empty):
            counter +=1
            shape_type, name, pos, height, rad = self.particle_prims.pop(0)
            prim_utils.delete_prim(Sdf.Path(self.path_prefix+name))
            # resetted prims go to the end so will not be popped
            if shape_type=="ball":
                self.create_powder_ball(pos, rad, name)
            elif shape_type=="cylinder":
                self.create_powder_cylinder(pos, height, rad, name)

    def create_powder_cylinder(self, position, height, radius, name=None):
        if name is None:
            name = "/Particles"+str(self.id_count)
        self.particle_prims.append(("cylinder", name, position, height, radius))
        self.id_count+=1
        particleSpacing = self.solidRestOffset*2
        points = []
        dim = math.ceil(2 * radius / particleSpacing)
        rows = math.ceil(height/particleSpacing)

        for i in range(dim):
            for j in range(dim):
                for k in range(rows):
                    x = i * particleSpacing - radius 
                    y = j * particleSpacing - radius 
                    z = k * particleSpacing - radius 

                    d2 = x * x + y * y 
                    if d2 < radius * radius:
                        points.append(Gf.Vec3f(x, y, z))
        print(f"--> Created {len(points)} particles")
        self.__create_point_prim(position, points, name)
    
    def __create_point_prim(self, position, points, name = "/Particles"):
        basePos = Gf.Vec3f(0.0, 0.0, 0.0) + position

        positions_list = [x + basePos for x in points]
        velocities_list = [Gf.Vec3f(0.0, 0.0, 0.0)] * len(positions_list)
        # color = Vt.Vec3fArray([self.colors[self._num_balls % self._num_colors]])

        particlePointsPath = Sdf.Path(self.path_prefix+name)
        print( particlePointsPath, self.particleSystemPath)
        self.particlePrim = particleUtils.add_physx_particleset_pointinstancer(
            self.stage,
            particlePointsPath,
            positions_list,
            velocities_list,
            self.particleSystemPath,
            self_collision=True,
            fluid=self.cfg.fluid,
            particle_group=0,
            particle_mass=self.cfg.mass,
            density=self.cfg.density,
            # widths=[0.05]*len(positions_list),
        )

        prototypeStr = str(particlePointsPath) + "/particlePrototype0"
        gprim = UsdGeom.Sphere.Define(self.stage, Sdf.Path(prototypeStr))
        # gprim.CreateDisplayColorAttr(Vt.Vec3fArray([255.0 ,20.0 ,147.0]))
        gprim.CreateRadiusAttr().Set(self.solidRestOffset)
        

    def get_positions(self, env_id):
        path_prefix = f"/World/envs/env_{env_id}"
        
        positions = []
        for primData in self.particle_prims:
            particlePointsPath = Sdf.Path(path_prefix+primData[1])
            prim = self.stage.GetPrimAtPath(particlePointsPath)

            instancer = UsdGeom.PointInstancer(prim)
            
            positions.append(instancer.GetPositionsAttr().Get())
        return positions
            