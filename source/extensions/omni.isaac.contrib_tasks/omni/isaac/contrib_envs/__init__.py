# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Module containing environments contributed by the community.


We use OpenAI Gym registry to register the environment and their default configuration file.
The default configuration file is passed to the argument "kwargs" in the Gym specification registry.
The string is parsed into respective configuration container which needs to be passed to the environment
class. This is done using the function :meth:`load_default_env_cfg` in the sub-module
:mod:`omni.isaac.orbit.utils.parse_cfg`.

Note:
    This is a slight abuse of kwargs since they are meant to be directly passed into the environment class.
    Instead, we remove the key :obj:`cfg_file` from the "kwargs" dictionary and the user needs to provide
    the kwarg argument :obj:`cfg` while creating the environment.

Usage:
    >>> import gym
    >>> import omni.isaac.contrib_envs
    >>> from omni.isaac.orbit_envs.utils.parse_cfg import load_default_env_cfg
    >>>
    >>> task_name = "Isaac-Contrib-<my-registered-env-name>-v0"
    >>> cfg = load_default_env_cfg(task_name)
    >>> env = gym.make(task_name, cfg=cfg)
"""


import gym  # noqa: F401
import os
import toml

# Conveniences to other module directories via relative paths
ORBIT_CONTRIB_ENVS_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
"""Path to the extension source directory."""

ORBIT_CONTRIB_ENVS_DATA_DIR = os.path.join(ORBIT_CONTRIB_ENVS_EXT_DIR, "data")
"""Path to the extension data directory."""

ORBIT_CONTRIB_ENVS_METADATA = toml.load(os.path.join(ORBIT_CONTRIB_ENVS_EXT_DIR, "config", "extension.toml"))
"""Extension metadata dictionary parsed from the extension.toml file."""

# Configure the module-level variables
__version__ = ORBIT_CONTRIB_ENVS_METADATA["package"]["version"]

##
# Manipulation
##

gym.register(
    id="Isaac-PickPlace-Franka-v0",
    entry_point="omni.isaac.contrib_envs.manipulation.pick_place:PickPlaceEnv",
    kwargs={"cfg_entry_point": "omni.isaac.contrib_envs.manipulation.pick_place:PickPlaceEnvCfg"},
)

##
# Manipulation
##

gym.register(
    id="Isaac-Grocery-Franka-v0",
    entry_point="omni.isaac.contrib_envs.grocery.grocery_v0:GroceryEnv",
    kwargs={"cfg_entry_point": "omni.isaac.contrib_envs.grocery.grocery_v0:GroceryEnvCfg"},
)

##
# Chemistry-Robotics
##

gym.register(
    id="Isaac-Chemistry-Franka8DoF-Robotiq-v0",
    entry_point="omni.isaac.contrib_envs.chemistry.chemistry_v0:ChemistryV0Env",
    kwargs={"cfg_entry_point": "omni.isaac.contrib_envs.chemistry.chemistry_v0:ChemistryV0EnvCfg"},
)

gym.register(
    id="Isaac-Chemistry-Franka8DoF-Robotiq-v1",
    entry_point="omni.isaac.contrib_envs.chemistry.chemistry_v1:ChemistryV1Env",
    kwargs={"cfg_entry_point": "omni.isaac.contrib_envs.chemistry.chemistry_v1:ChemistryV1EnvCfg"},
)

gym.register(
    id="Isaac-Chemistry-VialInsertion-Franka-v0",
    entry_point="omni.isaac.contrib_envs.chemistry.vial_insertion:VialInsertionEnv",
    kwargs={"cfg_entry_point": "omni.isaac.contrib_envs.chemistry.vial_insertion:VialInsertionEnvCfg"},
)

gym.register(
    id="Isaac-Chemistry-PowderWeighting-Franka7DoF-v0",
    entry_point="omni.isaac.contrib_envs.chemistry.powder_weighting:PowderWeightingEnv",
    kwargs={"cfg_entry_point": "omni.isaac.contrib_envs.chemistry.powder_weighting:PowderWeightingEnvCfg"},
)
