import numpy as np
from mujoco_py import MujocoException
from .base_muscled_env import BaseMuscledEnv


class MuscledReacherEnv(BaseMuscledEnv):

    def __init__(self):
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

        # Get environment observations
        ob = self._get_obs()

        # TODO: Implement reward function
        reward = 0

        # TODO: Implement done cases
        done = False

        # TODO: Populate info dict
        info = dict()

        return ob, reward, done, info

    def _get_obs(self):
        return np.concatenate([
            self.muscle_fatigues
        ])

    def reset_model(self):
        self._reset_muscles(self.muscle_count)
        self._reset_fatigues(self.muscle_count)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5
