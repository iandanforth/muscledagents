"""
Ant

Learn to control your muscles to make forward progress!
"""
from muscledagents.envs.mujoco import MuscledAntEnv


def main():
    env = MuscledAntEnv(apply_fatigue=True)

    # Set up the simulation parameters
    sim_duration = 60  # seconds
    frames_per_second = 50
    step_size = 1 / frames_per_second
    total_steps = int(sim_duration / step_size)

    for i in range(total_steps):
        env.step([10.0] * env.muscle_count)
        env.render()


if __name__ == '__main__':
    main()
