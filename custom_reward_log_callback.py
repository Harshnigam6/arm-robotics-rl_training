from stable_baselines3.common.callbacks import BaseCallback

class SubRewardCallback(BaseCallback):
    """
    Logs sub-rewards to TensorBoard.
    """
    def __init__(self, verbose=0):
        super().__init__(verbose)

    def _on_step(self) -> bool:
        # self.training_env is likely a VecEnv, which has a list of sub envs under 'envs'.
        subenvs = self.training_env.envs

        # 'self.locals["infos"]' is also a typical way to retrieve info for each sub-env at the current step
        infos = self.locals["infos"]  # List of info dicts, one per sub-env
        
        for i, info in enumerate(infos):
            # If your environment logs sub-rewards in info["sub_rewards"], you can do:
            if "sub_rewards" in info:
                sub_rewards = info["sub_rewards"]
                for key, val in sub_rewards.items():
                    self.logger.record(f"sub_rew/{key}_env{i}", val)

        return True
