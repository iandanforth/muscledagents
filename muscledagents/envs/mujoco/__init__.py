from gym.envs.registration import register
from .base_muscled_env import BaseMuscledEnv # NoQA
from .muscled_hopper import MuscledHopperEnv # NoQA
from .muscled_ant import MuscledAntEnv # NoQA
from .muscled_reacher import MuscledReacherEnv # NoQA

register(
    id='BaseMuscledEnv-v0',
    entry_point='muscledagents.envs.mujoco:BaseMuscledEnv',
)


register(
    id='MuscledAnt-v0',
    entry_point='muscledagents.envs.mujoco:MuscledAntEnv',
    max_episode_steps=1000,
    reward_threshold=6000.0
)

register(
    id='MuscledReacher-v0',
    entry_point='muscledagents.envs.mujoco:MuscledReacherEnv',
    max_episode_steps=1000,  # Task is reach & hold.
    reward_threshold=-150    # Baseline from PPO
)
