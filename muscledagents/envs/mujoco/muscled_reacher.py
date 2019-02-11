import math
import numpy as np
from .base_muscled_env import BaseMuscledEnv


def unit_vector(vector):
    """
    Returns the unit vector of the vector.
    https://stackoverflow.com/a/13849249/1775741
    """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """
    Returns the angle in degrees between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793

    https://stackoverflow.com/a/13849249/1775741
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    rad = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    return rad


class MuscledReacherEnv(BaseMuscledEnv):
    """
    This environment makes several changes to the OpenAI Gym Env 'Reacher-v2'
    on which it is based.

    Major Changes:
        - Actuation is through tendon-actuator pairs
        - There is a fatigue model applied to each model (via PyMuscle)
        - The models have been scaled up for greater consistency among
          MuJoCo models.
        - The max number of steps per trial has been increased to 1000.
          This changes the task to a reach & hold-position task.
          The original task could be 'solved' with the hand still in motion.
        - The hand-crafted energy penalty has been removed.
        - The reward criteria based on the above changes is now -150.
        - As with all MuscledX envs the observation space includes the
          current fatigue levels of the muscles.
    """

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
        # Get the angles of the first two joints
        theta = self.sim.data.qpos.flat[:2]
        fingertip_vec = self.get_body_com("fingertip_body")
        target_vec = self.get_body_com("target_body")
        # Get the distance to the target
        self.dist_vec = fingertip_vec - target_vec
        return np.concatenate([
            np.cos(theta),  # Why are we taking cos here?
            np.sin(theta),  # Why are we taking sin here?
            self.sim.data.qpos.flat[2:],  # Location of the target
            self.sim.data.qvel.flat[:2],  # Angular rotation of arm joints
            self.dist_vec,
            self.muscle_fatigues
        ])

    def reset_model(self):
        # Randomize arm joint angles and target position.
        shoulderRange = 100 * (math.pi / 180)  # +/-
        elbowRange = 172 * (math.pi / 180)  # +/-
        targetRange = 2  # Full circle working area radius

        # Restrict target placement area to arc of shoulder and len of arm
        unitV = np.array([1, 0])
        while True:
            self.goal = self.np_random.uniform(low=-targetRange, high=targetRange, size=2)
            angle = abs(angle_between(unitV, self.goal))
            mag = np.linalg.norm(self.goal)
            if (mag < 2) and (angle < shoulderRange):
                break

        # Qpos
        # Q - Refers to the generalized coordinate
        #     http://www.mujoco.org/book/computation.html#General
        # Pos - position
        # This corresponds to joints [shoulder, elbow, targetX, targetY]
        qpos = np.array([
            self.np_random.uniform(low=-shoulderRange, high=shoulderRange),
            self.np_random.uniform(low=-elbowRange, high=elbowRange),
            self.goal[0],
            self.goal[1],
        ])

        # Set all velocities to zero when starting over.
        qvel = np.zeros(self.model.nq)
        self.set_state(qpos, qvel)

        # Reset our muscles and locally stored fatigue values
        self._reset_muscles(self.muscle_count)
        self._reset_fatigues(self.muscle_count)
        return self._get_obs()

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5
