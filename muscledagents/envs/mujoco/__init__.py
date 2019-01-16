from gym.envs.registration import register
from .muscled_hopper import MuscledHopperEnv # NoQA
from .muscled_ant import MuscledAntEnv # NoQA


register(
    id='MuscledAnt-v0',
    entry_point='muscledagents.envs.mujoco:MuscledAntEnv',
)
