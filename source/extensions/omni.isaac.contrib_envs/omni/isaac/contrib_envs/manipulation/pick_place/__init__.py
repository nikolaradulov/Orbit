# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Environment for picking and placing objects with fixed-arm robots."""

from .pick_place_cfg import PickPlaceEnvCfg
from .pick_place_env import PickPlaceEnv

__all__ = ["PickPlaceEnv", "PickPlaceEnvCfg"]
