# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to create curriculum for the learning environment.

The functions can be passed to the :class:`omni.isaac.orbit.managers.CurriculumTermCfg` object to enable
the curriculum introduced by the function.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:
    from omni.isaac.orbit.envs import RLTaskEnv


def modify_reward_weight(env: RLTaskEnv, env_ids: Sequence[int], term_name: str, weight: float, num_steps: int):
    """Curriculum that modifies a reward weight a given number of steps.

    Args:
        env: The learning environment.
        env_ids: Not used since all environments are affected.
        term_name: The name of the reward term.
        weight: The weight of the reward term.
        num_steps: The number of steps after which the change should be applied.
    """
    if env.common_step_counter > num_steps:
        # obtain term settings
        term_cfg = env.reward_manager.get_term_cfg(term_name)
        # update term settings
        term_cfg.weight = weight
        env.reward_manager.set_term_cfg(term_name, term_cfg)
