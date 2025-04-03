import time
import numpy as np
import pybullet as p
from xarm_env import XArm6PickPlaceEnv

def manual_joint_control_with_reward_near_robot():
    """
    Sets up the XArm6PickPlaceEnv in render mode, creates debug sliders to directly control each robot joint,
    and displays the computed reward as text next to the robot. The reward text is updated every simulation step
    at a position relative to the robot's current position.
    """
    # Initialize environment in render mode.
    env = XArm6PickPlaceEnv(render=True)
    env.reset()  # Reset the simulation
    
    # Get the robot's PyBullet body id from the environment.
    robot_id = env.robot_id
    
    # Create sliders for each joint for manual control.
    num_joints = p.getNumJoints(robot_id)
    slider_ids = {}
    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot_id, joint_idx)
        joint_name = joint_info[1].decode("utf-8")
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        if lower_limit > upper_limit:  # In case limits are not properly set.
            lower_limit, upper_limit = -3.14, 3.14
        slider_ids[joint_idx] = p.addUserDebugParameter(joint_name, lower_limit, upper_limit, 0.0)
        print(f"Created slider for joint {joint_idx} ({joint_name}) with range [{lower_limit}, {upper_limit}].")
    
    # Define an offset so that the text appears near (e.g., above) the robot.
    offset = [0, 0, 0.5]  # Adjust as needed.
    
    # Variable to store the current debug text id.
    reward_text_id = None

    print("Use the joint sliders to control the robot. Reward text will be displayed next to the robot.")

    try:
        while True:
            # Update joint positions based on slider values.
            for joint_idx, slider_id in slider_ids.items():
                target_angle = p.readUserDebugParameter(slider_id)
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_angle,
                    force=500  # Adjust as needed.
                )
            
            # Step the simulation.
            p.stepSimulation()
            time.sleep(1 / 240)  # Match simulation timestep.
            
            # Compute the current reward from the environment.
            reward = env._compute_reward()
            
            # Get the robot's current position.
            # You can use getBasePositionAndOrientation for the base or getLinkState for a specific link.
            robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
            # Position where the reward text will be displayed (offset from the robot).
            display_pos = [robot_pos[i] + offset[i] for i in range(3)]
            
            # Remove the previous debug text, if it exists.
            if reward_text_id is not None:
                p.removeUserDebugItem(reward_text_id)
            # Add new debug text at the computed position.
            reward_text_id = p.addUserDebugText(
                f"Reward: {reward:.3f}",
                display_pos,
                textColorRGB=[1, 0, 0],
                textSize=1.5,
                lifeTime=0  # 0 means it persists until removed.
            )
    except KeyboardInterrupt:
        print("Exiting manual joint control with reward display.")
    finally:
        env.close()

if __name__ == "__main__":
    manual_joint_control_with_reward_near_robot()
