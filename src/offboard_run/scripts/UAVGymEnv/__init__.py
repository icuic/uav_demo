from gymnasium.envs.registration import register

register(
    id="UAVGymEnv/UAVLandingEnv-v0",
    entry_point="UAVGymEnv.envs:UAVLandingEnv",
)