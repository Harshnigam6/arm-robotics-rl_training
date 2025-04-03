import os
import time
import stable_baselines3 as sb3
from stable_baselines3.common.vec_env import DummyVecEnv
from xarm_env import XArm6PickPlaceEnv

def test_xarm_model(model_path: str, n_episodes: int = 1, max_steps: int = 200):
    """
    Loads a PPO model from the given path and runs it in the XArm6PickPlaceEnv
    for n_episodes episodes, each with up to max_steps steps.

    :param model_path: Path to the saved Stable Baselines 3 model (e.g., './checkpoints/xarm6_model_395000_steps.zip').
    :param n_episodes: Number of test episodes to run.
    :param max_steps: Max steps per episode (if not already constrained by a TimeLimit wrapper).
    """

    # 1) Load the trained model
    print(f"Loading model from: {model_path}")
    model = sb3.PPO.load(model_path)

    # 2) Create the environment (render=True to visualize, set to False if you want headless)
    #    If your environment already has an internal time limit, you can skip extra wrappers.
    env = XArm6PickPlaceEnv(render=True)

    # Wrap it in a DummyVecEnv for Stable Baselines if needed
    env = DummyVecEnv([lambda: env])

    # 3) Run the model for a certain number of episodes
    for episode_idx in range(n_episodes):
        obs = env.reset()
        done = [False]  # done is an array in a VecEnv
        episode_reward = 0.0
        step_count = 0

        while not done[0] and step_count < max_steps:
            # Use the model to predict an action
            action, _states = model.predict(obs, deterministic=True)
            print(action)
            
            # Step the environment
            obs, rewards, done, info = env.step(action)
            episode_reward += rewards[0]
            step_count += 1

            # Optional: if the environment needs an explicit render call
            # env.render()

            # Sleep a bit so that you can see the motion if it's too fast
            # time.sleep(0.01)

        print(f"Episode {episode_idx+1}/{n_episodes} finished. Reward: {episode_reward}")

    # 4) Close the environment
    env.close()


if __name__ == "__main__":
    # Example usage:
    # Replace with your actual checkpoint file name
    checkpoint_file = "./checkpoints/xarm6_model_480000_steps.zip"

    # Make sure the checkpoint file exists
    if not os.path.isfile(checkpoint_file):
        raise FileNotFoundError(f"Checkpoint file not found: {checkpoint_file}")

    # Run the test
    test_xarm_model(model_path=checkpoint_file, n_episodes=5, max_steps=200)
