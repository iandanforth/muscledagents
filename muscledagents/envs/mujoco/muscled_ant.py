import numpy as np
from mujoco_py import MujocoException
from .base_muscled_env import BaseMuscledEnv


class MuscledAntEnv(BaseMuscledEnv):

    def __init__(self):
        super().__init__(
            muscle_count=16,
            xml_filename='muscled-ant.xml'
        )

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

        # This model can randomly become unstable
        # Until this can be reproduced reliably this hack will
        # have to do.
        unstable = 0
        try:
            self.do_simulation(outputs, self.frame_skip)
        except MujocoException as e:
            print(e)
            unstable = 1

        # Calculate reward and see if we're done
        xposafter = self.get_body_com("torso")[0]
        forward_reward = (xposafter - xposbefore)/self.dt
        ctrl_cost = .5 * np.square(a).sum()
        # cfrc_ext is "com-based external force on body         (nbody x 6)"
        # I don't know what this means or why it is scaled
        # to this level.
        contact_cost = 0.5 * 0.001 * np.sum(
            np.square(np.clip(self.sim.data.cfrc_ext, -1, 1))
        )
        survive_reward = 1.0
        reward = forward_reward - ctrl_cost - contact_cost + survive_reward
        # TODO: Remove this
        reward = forward_reward - unstable

        # State is a flattened combination of qpos and qvel
        # Here it has 29 items
        state = self.state_vector()

        # Done conditions
        #   - Some value of state is infinite
        #   - The agent is not too low (z <= 0.2) or too high z >=1.0
        notdone = np.isfinite(state).all() and state[2] >= 0.2 and state[2] <= 1.0
        done = not notdone
        if unstable != 0:
            done = True

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
        self._reset_muscles(self.muscle_count)
        self._reset_fatigues(self.muscle_count)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5

    # def viewer_setup(self):
    #     self.viewer.cam.trackbodyid = 2
    #     self.viewer.cam.distance = self.model.stat.extent * 0.75
    #     self.viewer.cam.lookat[2] += .8
    #     self.viewer.cam.elevation = -20
