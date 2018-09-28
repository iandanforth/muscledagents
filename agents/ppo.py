import gym
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO1

env = gym.make('CartPole-v1')
env = DummyVecEnv([lambda: env])

model = PPO1(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=25000)
model.save("ppo1_cartpole")

del model  # remove to demonstrate saving and loading

model = PPO1.load("ppo1_cartpole")

obs = env.reset()
score = 0
max_trials = 100
trials = 0
scores = []
min_score = None
while trials < max_trials:
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    if done:
        print(score)
        scores.append(score)
        if min_score is None or score < min_score:
            min_score = score
        score = 0
        trials += 1
        env.reset()

    score += 1
    # env.render()

avg_score = sum(scores) / len(scores)
print("Average Score over {} trials was {}".format(max_trials, avg_score))
print("Lowest Score was {}".format(min_score))
