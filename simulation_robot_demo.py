import pybullet as p
import pybullet_data
import time

import pybullet as p
import pybullet_data
import time

def main():
    # Connect to simulation and set up environment
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load a plane and the xArm6 with gripper
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(
        "assets_repo/bullet3/examples/pybullet/gym/pybullet_data/xarm/xarm6_with_gripper.urdf",
        basePosition=[0, 0, 0],
        useFixedBase=True
    )

    # Get the total number of joints in the robot
    num_joints = p.getNumJoints(robot_id)

    # Create a global slider to adjust the joint force
    force_slider_id = p.addUserDebugParameter("Joint Force", 0.0, 100.0, 50.0)

    # Create a slider for each movable joint (revolute or prismatic)
    joint_sliders = {}
    for j_idx in range(num_joints):
        joint_info = p.getJointInfo(robot_id, j_idx)
        joint_type = joint_info[2]
        joint_name = joint_info[1].decode("utf-8")
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]

        # Only create sliders for joints that can move
        if joint_type not in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            continue

        # Use default limits if the URDF limits are not valid (e.g., for continuous joints)
        if upper_limit < lower_limit:
            if joint_type == p.JOINT_REVOLUTE:
                lower_limit, upper_limit = -3.14, 3.14
            else:
                lower_limit, upper_limit = -0.1, 0.1

        slider_id = p.addUserDebugParameter(
            paramName=f"{joint_name} (joint {j_idx})",
            rangeMin=lower_limit,
            rangeMax=upper_limit,
            startValue=0.0
        )
        joint_sliders[j_idx] = slider_id

    # --- Gripper Gear Constraint Integration ---
    # Specify the indices for the drive joint and the right outer knuckle joint.
    drive_joint_idx = 8
    right_outer_knuckle_idx = 11

    # Create a gear constraint between these two joints.
    constraint_id = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=drive_joint_idx,       # The "drive_joint"
        childBodyUniqueId=robot_id,
        childLinkIndex=right_outer_knuckle_idx,  # The "right_outer_knuckle_joint"
        jointType=p.JOINT_GEAR,
        jointAxis=[1, 0, 0],  # Rotation axis
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )

    # Set the gear ratio. A value of 1.0 means both joints rotate identically.
    p.changeConstraint(constraint_id, gearRatio=-1.0, maxForce=50.0)
    # --- End of Gripper Gear Constraint Integration ---

    # Main simulation loop: read slider values and control the joints accordingly.
    while True:
        # Read the overall force value from the debug slider
        force_val = p.readUserDebugParameter(force_slider_id)

        # Update each movable joint to the target positions from their respective sliders
        for j_idx, slider_id in joint_sliders.items():
            target_pos = p.readUserDebugParameter(slider_id)


            if j_idx == 11:
                # Freeing the 11 joint so that GUI sliders dont overide the gripper
                p.setJointMotorControl2(
                            robot_id,
                            j_idx,
                            p.VELOCITY_CONTROL,
                            targetVelocity=0.0,
                            force=0
                        )
            else:
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=j_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_pos,
                    force=force_val
                )

        p.stepSimulation()
        time.sleep(1. / 240.)

if __name__ == "__main__":
    main()
