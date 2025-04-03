import pybullet as p
import pybullet_data
import numpy as np
import time
import cv2
from camera_input import *


# Overhead camera config
cam_overhead = {
    'target': [0.5, 0.0, 0.0],  # What point is the camera aimed at?
    'distance': 0.7,           # How far from the target
    'yaw': 0,
    'pitch': -90,              # -90 => looking straight down
    'roll': 0,
    'upAxisIndex': 2,          # Z-axis is 'up'

    'width': 320,
    'height': 240,
    'fov': 60,                 # Field of view in degrees
    'near': 0.01,
    'far': 2.0
}

# Side/angled camera config
cam_side = {
    'target': [0.5, 0.0, 0.0],
    'distance': 1.0,
    'yaw': 45,
    'pitch': -30,              # angled downward
    'roll': 0,
    'upAxisIndex': 2,

    'width': 320,
    'height': 240,
    'fov': 60,
    'near': 0.01,
    'far': 3.0  # maybe a bit further so we can see more
}


# 1) Connect & set up
client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane_id = p.loadURDF("plane.urdf")

# Create tray
tray_half_extents = [0.2, 0.2, 0.005]
# tray_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=tray_half_extents)
# tray_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=tray_half_extents, rgbaColor=[0.8,0.6,0.4,1])
# tray_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tray_collision_shape,
#                             baseVisualShapeIndex=tray_visual_shape, basePosition=[0.5,0,0.025])

# Load xArm
xarm_urdf_path = "assets_repo/bullet3/examples/pybullet/gym/pybullet_data/xarm/xarm6_with_gripper.urdf"
xarm_id = p.loadURDF(xarm_urdf_path, basePosition=[0,0,0], useFixedBase=True)

# Create box
obj_half_extents = [0.02, 0.02, 0.02]
obj_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=obj_half_extents)
obj_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=obj_half_extents, rgbaColor=[1,0,0,1])
box_start_position = [0.5, 0, 0.1]
box_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=obj_collision_shape,
                           baseVisualShapeIndex=obj_visual_shape, basePosition=box_start_position)

# Wait a bit so the box settles
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

##########################
# Insert the functions here
##########################

def control_gripper(state, xarm_id):
    gripper_joints = [8, 11]  # Might need adjusting
    open_pos = 0.0
    close_pos = 0.8
    target_pos = open_pos if state == 'open' else close_pos
    for j in gripper_joints:
        p.setJointMotorControl2(xarm_id, 8, p.POSITION_CONTROL, targetPosition=target_pos, force=500)
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def move_arm_to_target(target_pos, target_orn, xarm_id):
    arm_joints = [1, 2, 3, 4, 5, 6]  # The 6 xArm joints
    end_effector_link = 6           # or 6, depending on your URDF

    # optional damping
    joint_damping = [0.1]*6  

    joint_angles = p.calculateInverseKinematics(
        xarm_id,
        end_effector_link,
        target_pos,
        target_orn,
        jointDamping=joint_damping
    )

    # Apply the resulting angles
    for i, j_idx in enumerate(arm_joints):
        p.setJointMotorControl2(
            xarm_id,
            j_idx,
            p.POSITION_CONTROL,
            targetPosition=joint_angles[i],
            force=500,
            maxVelocity=1
        )


    # Store joint positions
    joint_positions_log = [] 

    # Step simulation to let it move
    for _ in range(100):
        p.stepSimulation()


        # Record joint positions
        joint_positions = [p.getJointState(xarm_id, j_idx)[0] for j_idx in arm_joints]
        print(joint_positions)


        overhead_img = capture_image(cam_overhead)
        # overhead_img is a tuple: (width, height, rgbData, depthData, segData, ...)
        # Convert RGBA -> BGR for OpenCV display
        overhead_w = overhead_img[0]
        overhead_h = overhead_img[1]
        overhead_rgba = overhead_img[2]  # raw data
        overhead_rgba = np.reshape(overhead_rgba, (overhead_h, overhead_w, 4))
        overhead_bgr = overhead_rgba[..., :3][..., ::-1]  # strip alpha, flip R->B

        # Capture side camera
        side_img = capture_image(cam_side)
        side_w = side_img[0]
        side_h = side_img[1]
        side_rgba = side_img[2]
        side_rgba = np.reshape(side_rgba, (side_h, side_w, 4))
        side_bgr = side_rgba[..., :3][..., ::-1]

        # Display each camera in its own window
        cv2.imshow("Overhead Camera", overhead_bgr)
        cv2.imshow("Side Camera", side_bgr)

        #     # Important: allow OpenCV GUI to update
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



        joint_positions_log.append([joint_positions, overhead_bgr, side_bgr])


        time.sleep(1./240.)


def grasp_object(box_id, xarm_id):
    # Get object position
    obj_pos, obj_orn = p.getBasePositionAndOrientation(box_id)
    print(obj_pos, '---------------------------------------')
    pre_grasp_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.05]
    grasp_pos     = [obj_pos[0], obj_pos[1], obj_pos[2] - 0.02]
    target_orn = p.getQuaternionFromEuler([0, np.pi, 0])  # Adjust as needed

    # 1) Open
    control_gripper('open', xarm_id)
    # 2) Above
    move_arm_to_target(pre_grasp_pos, target_orn, xarm_id)
    # 3) Down
    move_arm_to_target(grasp_pos, target_orn, xarm_id)
    # 4) Close
    control_gripper('close', xarm_id)
    # 5) Lift
    move_arm_to_target(pre_grasp_pos, target_orn, xarm_id)



for i in range(p.getNumJoints(xarm_id)):
    info = p.getJointInfo(xarm_id, i)
    print(i, info[1], info[12])  # (index, jointName, linkName)
###################
# Call the grasp
###################
grasp_object(box_id, xarm_id)



# You can keep stepping the simulation or do other stuff
while True:
    p.stepSimulation()
    time.sleep(1./240.)
