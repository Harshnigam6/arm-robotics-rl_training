import stable_baselines3 as sb3
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
from stable_baselines3.common.callbacks import BaseCallback
from gym.wrappers import TimeLimit
from stable_baselines3.common.monitor import Monitor
from xarm_env import XArm6PickPlaceEnv
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList

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




# Create parallelized vectorized environment (e.g., 8 parallel envs)
num_envs = 2

def make_env(env_id, max_episode_steps=200):
    """
    Factory function to create a single env wrapped with a time limit.
    """
    def _init():
        # 1) Instantiate your custom env
        env = XArm6PickPlaceEnv(render=False)
        
        # 2) Wrap it with TimeLimit to cap episode length
        env = TimeLimit(env, max_episode_steps=max_episode_steps)

        # <--- wrap in Monitor so we get episode rewards
        env = Monitor(env)  
        
        return env
    return _init

envs = [make_env(i, max_episode_steps=200) for i in range(num_envs)]
# 2) Wrap them in a vectorized environment
envs = DummyVecEnv(envs)
envs = VecMonitor(envs)  # recommended for SB3
# env = DummyVecEnv([lambda: XArm6PickPlaceEnv(render=False) for _ in range(8)])

# Instantiate PPO with a MultiInputPolicy (for dict obs) and desired hyperparameters
model = sb3.PPO(
    policy="MultiInputPolicy", 
    env=envs, 
    learning_rate=3e-4, 
    n_steps=4096,            # rollout steps per environment before update (total batch = 4096*8 if 8 envs)
    batch_size=256,          # minibatch size for SGD (must be <= n_steps * n_envs)
    n_epochs=10,             # number of epochs to iterate over each batch
    gamma=0.99, 
    gae_lambda=0.95, 
    clip_range=0.2, 
    ent_coef=0.0,            # entropy coefficient (could tune >0 to encourage exploration)
    device="cuda",           # use GPU acceleration
    tensorboard_log="./ppo_xarm6_tensorboard/"
)


callback = SubRewardCallback()


checkpoint_callback = CheckpointCallback(
    save_freq=10000,               # Save the model every 10k timesteps
    save_path='./checkpoints/',    # Folder to save the checkpoint files
    name_prefix='xarm6_model'      # Prefix for the saved files
)


combined_callback = CallbackList([SubRewardCallback(), checkpoint_callback])

# Train the agent
model.learn(total_timesteps=1_000_000, callback=combined_callback)  # training for 1e6 timesteps as an example
model.save("xarm6_ppo_policy")
