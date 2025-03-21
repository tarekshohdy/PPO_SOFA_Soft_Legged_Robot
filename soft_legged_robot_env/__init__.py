from gymnasium.envs.registration import register

register(
    id="soft_legged_robot_env/SoftLegRobot-v0",
    entry_point="soft_legged_robot_env.envs:SoftLegRobotEnv",
)
