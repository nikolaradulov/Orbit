"""Environment for inserting a vial into a rack with fixed-arm robots."""

from .vial_insertion_cfg import VialInsertionEnvCfg
from .vial_insertion_env import VialInsertionEnv

__all__ = ["VialInsertionEnv", "VialInsertionEnvCfg"]