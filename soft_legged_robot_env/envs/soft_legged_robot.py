from enum import Enum
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from soft_legged_robot_env.envs.robot import init_simulation, run_simulation, reset_simulation

class Actions(Enum):
    cable1_pull = 0
    cable2_pull = 1
    cable3_pull = 2
    cable4_pull = 3
    cable5_pull = 4
    cable6_pull = 5
    cable7_pull = 6
    cable8_pull = 7
    cable9_pull = 8
    cable1_realease = 9
    cable2_realease = 10
    cable3_realease = 11
    cable4_realease = 12
    cable5_realease = 13
    cable6_realease = 14
    cable7_realease = 15
    cable8_realease = 16
    cable9_realease = 17

class SoftLegRobotEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 20}

    def __init__(self, render_mode=None):
        
        self._xy_min = 50
        self._xy_max = 100

        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Box(-np.inf, np.inf, shape=(6,), dtype=float),
                "target": spaces.Box(-self._xy_max, self._xy_max, shape=(2,), dtype=float),
            }
        )

        self._time_steps = 0
        self._max_steps = 1000

        self._cable_min = 0
        self._cable_max = 25

        self._vx_dir = 1
        self._vy_dir = 1

        self._reset_flag = True
        # We have 6 actions in total: pull/release each of the 9 cables.
        self.action_space = spaces.Box(self._cable_min, self._cable_max, shape=(9,), dtype=np.float32)
        """
        The following dictionary maps abstract actions from `self.action_space` to 
        the direction we will walk in if that action is taken.
        i.e. 0 corresponds to "right", 1 to "up" etc.
        """

        self._cable_inputs = np.zeros(9)
        
        self._distance_to_target = None
        self.distance_threshold = 20
        self._roll_max = np.deg2rad(40)
        self._pitch_max = np.deg2rad(40)
        self.height_min = -40
        init_simulation()

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        """
        If human-rendering is used, `self.window` will be a reference
        to the window that we draw to. `self.clock` will be a clock that is used
        to ensure that the environment is rendered at the correct framerate in
        human-mode. They will remain `None` until human-mode is used for the
        first time.
        """
        self.window = None
        self.clock = None

    def _get_action_cable(self, action):
        direction = np.zeros(9)
        if action < 9:
            direction[action] = 1
        else:
            direction[action-9] = -1
        return direction
    
    def _get_vxy_dir(self):
        self._vx_dir = np.round(((self._target_location[0] - self._agent_location[0]) / 
                                    abs(self._target_location[0] - self._agent_location[0] + 1e-6)))
        self._vy_dir = np.round(((self._target_location[1] - self._agent_location[1]) / 
                                    abs(self._target_location[1] - self._agent_location[1] + 1e-6)))
        
        return (self._vx_dir, self._vy_dir)
    
    def _get_obs(self):
        return {"agent": self._agent_location, "target": self._target_location}

    def _get_info(self):
        return {
            "distance": np.linalg.norm(
                self._agent_location[:2] - self._target_location
            ),
            "time_steps": self._time_steps,
            "cable_inputs": self._cable_inputs,
            "height": self._agent_location[2],
            "roll": self._agent_location[3],
            "pitch": self._agent_location[4],
            "yaw": self._agent_location[5],
            "velocity": self._agent_velocity,
            "v_dir": (self._vx_dir, self._vy_dir)
        }

    def reset(self, goal=None, reset = False , seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # choose a random target location
        if goal is None:
            self._target_location = self.np_random.uniform(self._xy_min, self._xy_max, size=(2,))

            if np.random.rand() < 0.5:
                self._target_location[0] *= -1
            
            if np.random.rand() < 0.5:
                self._target_location[1] *= -1
        
        else:
            self._target_location = np.array(goal)

        self._vx_dir = 1
        self._vy_dir = 1

        if self._reset_flag or reset:
        # the agent's location at inputs [0, 0, 0]
            self._agent_location, self._agent_velocity = reset_simulation()
            self._reset_flag = False

        self._distance_to_target = np.linalg.norm(
            self._agent_location[:2] - self._target_location
        )

        self._vx_dir, self._vy_dir = self._get_vxy_dir()

        self._cable_inputs = np.zeros(9)

        self._time_steps = 0

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, action: list):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        self._time_steps += 1

        # direction = self._get_action_cable(action)

        # We use `np.clip` to make sure we don't leave the grid
        self._cable_inputs = np.array(action)
        self._cable_inputs = np.clip(self._cable_inputs, self._cable_min, self._cable_max)

        # print(self._cable_inputs)

        self._agent_location, self._agent_velocity = run_simulation(self._cable_inputs)

        self._distance_to_target = np.linalg.norm(
            self._agent_location[:2] - self._target_location
        )

        height = self._agent_location[2]
        roll = self._agent_location[3]
        pitch = self._agent_location[4]

        terminated = False
        truncated = False

        self._vx_dir, self._vy_dir = self._get_vxy_dir()

        # reward = (1 - (np.log(1 + self._distance_to_target)/np.log(1 + self.distance_threshold))) - abs(roll) - abs(pitch)
        reward = (( 25 * (1 - (np.log(1 + self._distance_to_target)/np.log(1 + self.distance_threshold))))
                    - abs(np.rad2deg(roll))
                    - abs(np.rad2deg(pitch))
                    + self._agent_velocity[0] * self._vx_dir 
                    + self._agent_velocity[1] * self._vy_dir )
        
        # print(self._vx_dir , self._vy_dir)
        observation = self._get_obs()
        info = self._get_info()

        if self._distance_to_target < self.distance_threshold:
            reward += 1000
            terminated = True

        if (height < self.height_min or abs(roll) > self._roll_max or abs(pitch) > self._pitch_max):
            self._reset_flag = True
            truncated = True

        if self._time_steps >= self._max_steps:
            truncated = True

        return observation, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self):
        pass

    def close(self):
        pass
