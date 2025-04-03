import pybullet as p
import time
import numpy as np



############################################## openAI
# Adjust these to match your URDF
ARM_JOINT_INDICES = [1, 2, 3, 4, 5, 6]
END_EFFECTOR_LINK_INDEX = 6   # Typically the wrist link
GRIPPER_DRIVE_JOINT = 8       # 'drive_joint'

def pick_and_place(
    robot_id,
    box_position,
    end_effector_index=END_EFFECTOR_LINK_INDEX,
    arm_joints=ARM_JOINT_INDICES,
    drive_joint=GRIPPER_DRIVE_JOINT
):
    """
    Moves the xArm gripper above 'box_position', closes the gripper,
    and lifts the box.
    """
    ##################
    # 1. Hover Pose
    ##################
    hover_height = 0.12
    pick_orientation = p.getQuaternionFromEuler([0, np.pi, 0])
    hover_position = [box_position[0], box_position[1], hover_height]

    # Solve IK
    hover_joint_positions = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        hover_position,
        pick_orientation
    )

    # Apply to the arm
    for i, j_idx in enumerate(arm_joints):
        # i-th value from IK is for joint i, but your URDF might have them in the same order.
        # Usually it's the same ordering: joint1 -> index 1, joint2 -> index 2, etc.
        p.setJointMotorControl2(
            robot_id,
            j_idx,
            p.POSITION_CONTROL,
            targetPosition=hover_joint_positions[i],
            force=100
        )

    # Step simulation
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1/240.)

    ##################
    # 2. Move Down
    ##################
    pick_position = [box_position[0], box_position[1], 0.06]
    pick_joint_positions = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        pick_position,
        pick_orientation
    )

    for i, j_idx in enumerate(arm_joints):
        p.setJointMotorControl2(
            robot_id,
            j_idx,
            p.POSITION_CONTROL,
            targetPosition=pick_joint_positions[i],
            force=100
        )

    for _ in range(240):
        p.stepSimulation()
        time.sleep(1/240.)

    ##################
    # 3. Close Gripper
    ##################
    # If your drive_joint range is 0 to ~0.4 or 0.8, test & see what angle fully closes
    CLOSE_ANGLE = 0.4
    p.setJointMotorControl2(
        robot_id,
        drive_joint,
        p.POSITION_CONTROL,
        targetPosition=CLOSE_ANGLE,
        force=20
    )

    for _ in range(240):
        p.stepSimulation()
        time.sleep(1/240.)

    ##################
    # 4. Lift the Box
    ##################
    lift_position = [box_position[0], box_position[1], 0.2]
    lift_joint_positions = p.calculateInverseKinematics(
        robot_id,
        end_effector_index,
        lift_position,
        pick_orientation
    )

    for i, j_idx in enumerate(arm_joints):
        p.setJointMotorControl2(
            robot_id,
            j_idx,
            p.POSITION_CONTROL,
            targetPosition=lift_joint_positions[i],
            force=100
        )

    for _ in range(240):
        p.stepSimulation()
        time.sleep(1/240.)

    print("Pick-and-place done!")




    ######################## deepseek

import pybullet as p
import numpy as np
import time

def control_gripper(state, xarm_id):
    """Control the gripper to open or close."""
    gripper_joints = [8, 11]  # Left and right outer knuckle joints
    
    # Adjust these values based on your URDF's joint limits
    open_pos = 0.0
    close_pos = 0.8
    
    target_pos = open_pos if state == 'open' else close_pos
    
    for j in gripper_joints:
        p.setJointMotorControl2(
            xarm_id,
            j,
            p.POSITION_CONTROL,
            targetPosition=target_pos,
            force=500  # Adjust force as needed
        )
    
    # Allow time for gripper to move
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def move_arm_to_target(target_pos, target_orn, xarm_id):
    """Move arm to target position and orientation using IK."""
    arm_joints = [1, 2, 3, 4, 5, 6]  # xArm6 main joints (verify indices!)
    end_effector_link = 7  # Gripper base link
    
    # Get current joint positions
    current_positions = [p.getJointState(xarm_id, j)[0] for j in arm_joints]
    
    # Calculate IK
    joint_angles = p.calculateInverseKinematics(
        xarm_id,
        end_effector_link,
        target_pos,
        target_orn,
        jointDamping=[0.1]*6,
        maxNumIterations=100,
        residualThreshold=1e-5,
        currentPositions=current_positions,
        jointList=arm_joints
    )
    
    # Apply joint angles
    for i, j in enumerate(arm_joints):
        p.setJointMotorControl2(
            xarm_id,
            j,
            p.POSITION_CONTROL,
            targetPosition=joint_angles[i],
            force=500,
            maxVelocity=1  # Adjust velocity as needed
        )
    
    # Allow time for movement
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def grasp_object(box_id, xarm_id):
    """Execute grasping sequence."""
    # Get object position
    obj_pos, obj_orn = p.getBasePositionAndOrientation(box_id)
    
    # Calculate positions
    pre_grasp_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.05]  # 5cm above
    grasp_pos = [obj_pos[0], obj_pos[1], obj_pos[2] - 0.02]  # Adjust for gripper geometry
    
    # Target orientation (gripper pointing down)
    target_orn = p.getQuaternionFromEuler([0, -np.pi/2, 0])  # Adjust if needed
    
    # Execute sequence
    control_gripper('open', xarm_id)
    move_arm_to_target(pre_grasp_pos, target_orn, xarm_id)
    move_arm_to_target(grasp_pos, target_orn, xarm_id)
    control_gripper('close', xarm_id)
    move_arm_to_target(pre_grasp_pos, target_orn, xarm_id)

# Add this after your existing setup code:
# Run the grasping sequence
# grasp_object()

# # Keep simulation running
# while True:
#     p.stepSimulation()
#     time.sleep(1./240.)
