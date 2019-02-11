import numpy as np
from gym import utils, spaces
from . import mujoco_env
from pymuscle import StandardMuscle as Muscle


class BaseMuscledEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self, muscle_count=0, xml_filename='empty-world.xml'):

        # Create the muscles for this env
        self.muscle_count = muscle_count
        self._reset_muscles(self.muscle_count)
        self._reset_fatigues(self.muscle_count)

        # Initialize first parent
        mujoco_env.MujocoEnv.__init__(self, xml_filename, 4)

        # Overwrite the action space
        # PyMuscles provide an abstraction over the underlying sim
        # We want to specify an input to the pymuscles and let them handle
        # inputs to the simulated actuators.
        low = np.zeros(self.muscle_count)
        high = np.ones(self.muscle_count)
        self.action_space = spaces.Box(low=low, high=high)

        # Initialize second parent
        utils.EzPickle.__init__(self)

    def _get_muscles(
        self,
        muscle_count,
        max_force=100.0
    ):
        """
        Create N muscles where N is the number of actuators specified in the
        model file.

        Note: max_force should be kept in line with the gainprm on actuators
        in the <xml_filename>.xml
        """
        muscles = []
        for i in range(muscle_count):
            muscle = Muscle(max_force)
            muscles.append(muscle)

        return muscles

    def _reset_muscles(self, muscle_count):
        if hasattr(self, "muscles"):
            del self.muscles
        self.muscles = self._get_muscles(muscle_count)

    def _reset_fatigues(self, muscle_count):
        self.muscle_fatigues = np.zeros(muscle_count)

    def step(self, a):

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
