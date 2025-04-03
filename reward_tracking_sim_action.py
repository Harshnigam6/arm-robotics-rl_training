import time
import numpy as np
import pybullet as p
from xarm_env import XArm6PickPlaceEnv

def manual_control_simulation():
    """
    This simulation creates an instance of your XArm6PickPlaceEnv in render mode.
    It adds debug sliders (controlled by the mouse) that map directly to the actions.
    Each simulation step prints the total reward (and sub-rewards if available) to the terminal.
    """
    # Create the environment with rendering enabled.
    env = XArm6PickPlaceEnv(render=True)
    obs = env.reset()
    
    # Retrieve the dimension of the action space.
    action_dim = env.action_space.shape[0]
    
    # Create a slider for each action dimension.
    # The range (-1.0, 1.0) and initial value (0.0) might need to be adjusted
    # based on the valid action values for your robot.
    slider_ids = []
    for i in range(action_dim):
        slider_id = p.addUserDebugParameter(f"Action_{i}", -1.0, 1.0, 0.0)
        slider_ids.append(slider_id)
    
    print("Adjust the sliders (using your mouse) to manually control the robot arm.")
    print("Reward values will be printed to the terminal in real time.")
    
    try:
        while True:
            # Read slider values and build the action vector.
            action = np.array([p.readUserDebugParameter(sid) for sid in slider_ids], dtype=np.float32)
            
            # Step the environment using the manually set action.
            obs, reward, done, info = env.step(action)
            
            # Print the computed total reward.
            print(f"Total Reward: {reward:.3f}")
            
            # If the environment provides sub-rewards in the info dict, print them too.
            if "sub_rewards" in info:
                print("Sub-rewards:", info["sub_rewards"])
            
            # Slow down the loop slightly so you can observe changes.
            time.sleep(0.1)
            
            # If the episode ends, reset the environment.
            if done:
                print("Episode finished. Resetting environment...")
                obs = env.reset()
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        env.close()

if __name__ == "__main__":
    manual_control_simulation()
