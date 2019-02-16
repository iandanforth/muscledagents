import numpy as np
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
            outputs.append(output)
        self.muscle_fatigues = np.array(fatigues)

        # Step the physics sim
        actuator_inputs = np.array(outputs) * -1   # Invert for MuJoCo
        self.do_simulation(actuator_inputs, self.frame_skip)

        # Calculate reward and see if we're done
        xposafter = self.get_body_com("torso")[0]
        forward_reward = (xposafter - xposbefore) / self.dt
        reward = forward_reward

        # State is a flattened combination of qpos and qvel
        # Here it has 29 items
        state = self.state_vector()

        # Done conditions
        #   - Some value of state is infinite
        #   - The agent is not too low (z <= 0.2) or too high z >=1.0
        notdone = np.isfinite(state).all() and state[2] >= 0.2 and state[2] <= 1.0
        done = not notdone

        # Get environment observations
        ob = self._get_obs()

        return ob, reward, done, dict(reward_forward=forward_reward)

    def _get_obs(self):
        return np.concatenate([
            self.sim.data.qpos.flat[2:],
            self.sim.data.qvel.flat,
            np.clip(self.sim.data.cfrc_ext, -1, 1).flat,
            self.muscle_fatigues
        ])

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(
            size=self.model.nq,
            low=-0.1,
            high=0.1
        )
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
