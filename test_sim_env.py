# test_env_manual_close.py
import time
import numpy as np
from xarm_env import XArm6PickPlaceEnv

def main():
    # 1) Create environment with GUI
    env = XArm6PickPlaceEnv(render=True)
    
    # 2) Reset environment (loads objects, sets initial positions)
    obs = env.reset()

    print("Simulation running... Close the PyBullet window or press Ctrl+C to end.")

    try:
        # 3) Keep simulation running indefinitely
        while True:
            # Step the simulation with a dummy action (or all zeros).
            # Adjust the array size if your step() expects more/less dimensions.
            dummy_action = np.zeros(6, dtype=np.float32)
            obs, reward, done, info = env.step(dummy_action)
            # print("========================================================================")

            # print(obs)
            # print(reward)

            # print("========================================================================")

            # If your environment’s step() never sets 'done=True', this loop continues forever.
            if done:
                obs = env.reset()

            # Slow the simulation for real-time viewing (optional)
            time.sleep(1.0 / 240.0)

    except KeyboardInterrupt:
        # Graceful exit if user presses Ctrl+C
        print("Exiting simulation.")

    finally:
        # 4) Close the environment and PyBullet connection
        env.close()

if __name__ == "__main__":
    main()
