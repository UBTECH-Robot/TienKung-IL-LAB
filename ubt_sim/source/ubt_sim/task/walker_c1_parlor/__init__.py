import gymnasium as gym

gym.register(
    id="UBTSim-WalkerC1-Parlor-v0",
    entry_point="ubt_sim.env:ManagerBasedRLDigitalTwinEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.walker_c1_parlor_env_cfg:WalkerC1ParlorEnvCfg",
    },
)
