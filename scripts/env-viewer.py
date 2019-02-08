import sys
import math
import muscledagents # NoQA
import gym


def main():

    env_name = sys.argv[1]

    # Load a standard ant
    env = gym.make(env_name)
    env.reset()

    print("Observation Space Dims", env.observation_space.shape)
    print("Action Space Dims", env.action_space.shape)

    action_size = env.action_space.shape[0]

    # Set up the simulation parameters
    sim_duration = 360  # seconds
    frames_per_second = 50
    step_size = 1 / frames_per_second
    total_steps = int(sim_duration / step_size)

    # Step according to a complex pattern
    # https://www.desmos.com/calculator/c0uq1mul2a
    action = [0.0] * action_size
    print("Cycling one muscle ...")
    for i in range(total_steps):
        if len(action) > 1:
            action[1] = ((math.sin(i / 25) + 1) / 2)
        ob, reward, done, extras = env.step(action)
        if i % 500 == 0:
            print("Resetting Env")
            env.reset()
        env.render()


if __name__ == '__main__':
    main()
