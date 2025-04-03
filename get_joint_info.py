# import pybullet as p
# import pybullet_data
# import time

# def main():
#     # Connect to PyBullet (GUI or DIRECT)
#     physics_client = p.connect(p.DIRECT)
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())
#     p.setGravity(0, 0, -9.81)

#     # Load plane
#     plane_id = p.loadURDF("plane.urdf")

#     # Load xArm6 URDF
#     robot_id = p.loadURDF(
#         "assets_repo/bullet3/examples/pybullet/gym/pybullet_data/xarm/xarm6_with_gripper.urdf",
#         basePosition=[0, 0, 0],
#         useFixedBase=True
#     )

#     # Print basic body info for completeness
#     body_info = p.getBodyInfo(robot_id)
#     print("Body Info (base link name, etc.):", body_info)

#     # Number of joints
#     num_joints = p.getNumJoints(robot_id)
#     print("Number of Joints:", num_joints)

#     # Iterate over each joint and print info
#     # PyBullet's getJointInfo returns a tuple with many fields:
#     # (jointIndex, jointName, jointType, qIndex, uIndex, flags, jointDamping,
#     #  jointFriction, jointLowerLimit, jointUpperLimit, jointMaxForce,
#     #  jointMaxVelocity, linkName, axis, parentFramePos, parentFrameOrn, parentIndex)
#     for i in range(num_joints):
#         joint_info = p.getJointInfo(robot_id, i)

#         joint_index = joint_info[0]
#         joint_name = joint_info[1].decode("utf-8")  # Convert from bytes
#         joint_type = joint_info[2]
#         link_name = joint_info[12].decode("utf-8")

#         # Check if the joint is fixed or movable
#         # PyBullet Joint Types: 0=REVOLUTE, 1=PRISMATIC, 2=SPHERICAL, 3=PLANAR, 4=FIXED
#         if joint_type == p.JOINT_FIXED:
#             joint_type_str = "FIXED"
#         elif joint_type == p.JOINT_REVOLUTE:
#             joint_type_str = "REVOLUTE"
#         elif joint_type == p.JOINT_PRISMATIC:
#             joint_type_str = "PRISMATIC"
#         else:
#             joint_type_str = f"Type {joint_type}"

#         print(f"Joint Index: {joint_index}")
#         print(f"  Name: {joint_name}")
#         print(f"  Link Name: {link_name}")
#         print(f"  Joint Type: {joint_type_str}")
#         print(f"  Lower Limit: {joint_info[8]}")
#         print(f"  Upper Limit: {joint_info[9]}")
#         print(f"  Max Force: {joint_info[10]}")
#         print(f"  Max Velocity: {joint_info[11]}")
#         print("")

#     # Disconnect (only for a standalone script)
#     p.disconnect()


# if __name__ == "__main__":
#     main()


##### Checking if there is a drive join

import pybullet as p
import pybullet_data
import time

def main():
    # Connect to PyBullet (GUI or DIRECT)
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load plane
    plane_id = p.loadURDF("plane.urdf")

    # Load the xArm6 + gripper
    robot_id = p.loadURDF(
        "assets_repo/bullet3/examples/pybullet/gym/pybullet_data/xarm/xarm6_with_gripper.urdf",
        basePosition=[0, 0, 0],
        useFixedBase=True
    )

    # Joints for the xArm gripper in your listing
    drive_joint_idx = 8
    gripper_joint_indices = [8, 9, 10, 11, 12, 13]

    # 1. Set the drive joint to 0.0 (fully open), step the simulation,
    #    and record positions of all gripper joints as "before."
    p.setJointMotorControl2(
        robot_id, drive_joint_idx,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.0,
        force=50
    )
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

    before_positions = []
    for j_idx in gripper_joint_indices:
        pos, vel, _, _ = p.getJointState(robot_id, j_idx)
        before_positions.append(pos)

    # 2. Now set only the drive_joint to 0.85 (close),
    #    step simulation, then record "after" positions of all joints.
    p.setJointMotorControl2(
        robot_id, drive_joint_idx,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.85,
        force=50
    )
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

    after_positions = []
    for j_idx in gripper_joint_indices:
        pos, vel, _, _ = p.getJointState(robot_id, j_idx)
        after_positions.append(pos)

    # Print side-by-side comparison
    print("Joint Index | Before   | After")
    for j_idx, before_pos, after_pos in zip(gripper_joint_indices, before_positions, after_positions):
        print(f"{j_idx:11d} | {before_pos:8.3f} | {after_pos:8.3f}")

    # Disconnect if desired
    # p.disconnect()

if __name__ == "__main__":
    main()
