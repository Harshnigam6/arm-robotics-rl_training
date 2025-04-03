import time
import numpy as np
import pybullet as p


## IMPORTANT NOTES
## To run this script succesfuly, we have to remove self.max_xyz_step
## from env from action space in step function


# Adjust the import below based on where your XArm6PickPlaceEnv class is defined.
# For example, if your class is in a module named "xarm_env.py", use:
# from xarm_env import XArm6PickPlaceEnv
from xarm_env import XArm6PickPlaceEnv

def generate_trajectory(env, num_move_steps=10, num_grip_steps=20):
    """
    Generate a fine-grained trajectory that moves the end-effector above the cube.
    
    The target is defined as [0.5, 0, 0.15]:
      - x = 0.5 aligns with the cube's x-coordinate.
      - z = 0.15 sets the end-effector above the cube.
    
    The function computes the required small increments from the current end-effector pose to the target.
    In Phase 1 the arm is moved to the target while keeping the gripper open (gripper action = 0).
    In Phase 2 the gripper gradually closes, interpolating from 0 to 0.85.
    """
    # Get current end-effector position from the environment.
    ee_state = p.getLinkState(env.robot_id, env.ee_link_index)
    current_pos = np.array(ee_state[0])
    target_pos = np.array([0.5, 0.0, 0.15])
    
    # Total displacement needed.
    delta = target_pos - current_pos
    # Compute the action increment per step.
    # Recall: env.step() computes: target_pos = current_pos + (action * max_xyz_step)
    # We want the sum of increments to equal 'delta', so each step's action is:
    action_increment = delta / (num_move_steps * env.max_xyz_step) # env.max_xyz_step
    
    trajectory = []
    # Phase 1: Move the end-effector to the target above the cube.
    for _ in range(num_move_steps):
        # Only change the position; keep the gripper open (gripper action = 0)
        action = np.array([
            action_increment[0], 
            action_increment[1], 
            action_increment[2],
            0.0]
            )
        trajectory.append(action)
    

    # Phase 2: Gradually close the gripper while holding position.
    for i in range(num_grip_steps):
        # Linearly interpolate the gripper action from open (0.0) to closed (0.85)
        grip_action = 0.85 * ((i + 1) / num_grip_steps)
        action = np.array([0.0, 0.0, 0.0, grip_action])
        trajectory.append(action)
        
    return trajectory

def test_env():
    # Create the environment with rendering enabled so you can observe the simulation.
    env = XArm6PickPlaceEnv(render=True)
    
    # Reset the environment to initialize simulation, objects, and cameras.
    obs = env.reset()
    
    # Generate a test trajectory.
    trajectory = generate_trajectory(env)
    
    # Define simulation time step.
    # Note: p.stepSimulation() advances the simulation by one time step.
    # The actual physics timestep might be 1/240 sec (common in PyBullet).
    sim_time_step = 0.3#1. / 240
    
    # Execute each action in the trajectory.
    for idx, action in enumerate(trajectory):
        ee_state = p.getLinkState(env.robot_id, env.ee_link_index)
        print(np.array(ee_state[0]), "current position", env.ee_link_index)
        obs, reward, done, info = env.step(action)
        print(action)
        # print(f"Step {idx+1:02d} | Action: {action} | Reward: {reward:.3f}")
        # Sleep to match real-time simulation (if needed).
        time.sleep(sim_time_step)
    
    # Hold the final state for observation.
    print("Trajectory complete. Holding final state for observation...")
    time.sleep(3)
    
    # Close the environment.
    env.close()

if __name__ == '__main__':
    test_env()
