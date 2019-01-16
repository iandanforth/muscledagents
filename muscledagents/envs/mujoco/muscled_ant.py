import numpy as np
from gym import utils
from . import mujoco_env
from pymuscle import StandardMuscle as Muscle


class MuscledAntEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):

        # Create the muscles for this ant
        self.muscle_count = 16
        self.muscles = self._get_muscles(self.muscle_count)
        self.muscle_fatigues = np.zeros(self.muscle_count)

        # Initialize parents
        mujoco_env.MujocoEnv.__init__(self, 'muscled-ant.xml', 4)
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
        in the muscled-ant.xml
        """
        muscles = []
        for i in range(muscle_count):
            muscle = Muscle(max_force)
            muscles.append(muscle)

        return muscles

    def step(self, a):
        xposbefore = self.get_body_com("torso")[0]

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

        # Calculate reward and see if we're done
        xposafter = self.get_body_com("torso")[0]
        forward_reward = (xposafter - xposbefore)/self.dt
        ctrl_cost = .5 * np.square(a).sum()
        contact_cost = 0.5 * 1e-3 * np.sum(
            np.square(np.clip(self.sim.data.cfrc_ext, -1, 1)))
        survive_reward = 1.0
        reward = forward_reward - ctrl_cost - contact_cost + survive_reward
        state = self.state_vector()
        notdone = np.isfinite(state).all() \
            and state[2] >= 0.2 and state[2] <= 1.0
        done = not notdone

        # Get environment observations
        ob = self._get_obs()

        return ob, reward, done, dict(
            reward_forward=forward_reward,
            reward_ctrl=-ctrl_cost,
            reward_contact=-contact_cost,
            reward_survive=survive_reward)

    def _get_obs(self):
        return np.concatenate([
            self.sim.data.qpos.flat[2:],
            self.sim.data.qvel.flat,
            np.clip(self.sim.data.cfrc_ext, -1, 1).flat,
            self.muscle_fatigues
        ])

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(size=self.model.nq, low=-.1, high=.1)
        qvel = self.init_qvel + self.np_random.randn(self.model.nv) * .1
        self.set_state(qpos, qvel)
        self.muscle_fatigues = np.zeros(self.muscle_count)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5

    # def viewer_setup(self):
    #     self.viewer.cam.trackbodyid = 2
    #     self.viewer.cam.distance = self.model.stat.extent * 0.75
    #     self.viewer.cam.lookat[2] += .8
    #     self.viewer.cam.elevation = -20
