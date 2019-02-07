import numpy as np
from mujoco_py import MujocoException
from .base_muscled_env import BaseMuscledEnv


class MuscledReacherEnv(BaseMuscledEnv):

    def __init__(self):

        # Values to be populated on first step()
        self.fingertip_vec = None
        self.target_vec = None
        self.dist_vec = None

        super().__init__(
            muscle_count=4,
            xml_filename='muscled-reacher.xml'
        )

    def step(self, a):
        # Step the muscle sims
        # Collect output and fatigue levels
        outputs = []
        fatigues = []
        for i, muscle in enumerate(self.muscles):
            output = muscle.step(a[i], 0.002 * self.frame_skip)
            fatigues.append(muscle.get_peripheral_fatigue())
            output = -1 * output
            outputs.append(output)
        self.muscle_fatigues = np.array(fatigues)

        # Step the physics sim
        outputs = np.array(outputs)

        self.do_simulation(outputs, self.frame_skip)

        # Observation
        ob = self._get_obs()

        # Reward
        # Note that all ctrl penalties are removed in muscled envs
        # self.dist_vec is updated in _get_obs() above
        reward = -np.linalg.norm(self.dist_vec)

        # Done
        # There are no termination conditions in this env
        done = False

        # Info
        info = dict(reward=reward)

        return ob, reward, done, info

    def _get_obs(self):
        theta = self.sim.data.qpos.flat[:2]
        fingertip_vec = self.get_body_com("fingertip_body")
        target_vec = self.get_body_com("target_body")
        self.dist_vec = fingertip_vec - target_vec
        return np.concatenate([
            np.cos(theta),
            np.sin(theta),
            self.sim.data.qpos.flat[2:],
            self.sim.data.qvel.flat[:2],
            self.dist_vec,
            self.muscle_fatigues
        ])

    def reset_model(self):
        self._reset_muscles(self.muscle_count)
        self._reset_fatigues(self.muscle_count)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5
