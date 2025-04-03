import pybullet as p
import pybullet_data
import time
import cv2
import numpy as np
from camera_input import *
from ik_movement import *


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

# Connect to PyBullet in GUI mode so we can see what’s going on.
client = p.connect(p.GUI)

# Let’s set the search path so we can load some default URDFs (plane, simple objects, etc.)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Gravity is probably good to have
p.setGravity(0, 0, -9.81)



plane_id = p.loadURDF("plane.urdf")



# Create tray
tray_half_extents = [0.2, 0.2, 0.05]  # half extents in x,y,z
tray_collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=tray_half_extents
)
tray_visual_shape = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=tray_half_extents,
    rgbaColor=[0.8, 0.6, 0.4, 1.0]  # some color
)
tray_id = p.createMultiBody(
    baseMass=0,  # 0 means fixed
    baseCollisionShapeIndex=tray_collision_shape,
    baseVisualShapeIndex=tray_visual_shape,
    basePosition=[0.5, 0, 0.025]  # place it in front of the robot
)


# Suppose you have xarm6_robot.urdf in some folder
xarm_urdf_path = "assets_repo/bullet3/examples/pybullet/gym/pybullet_data/xarm/xarm6_with_gripper.urdf"

xarm_id = p.loadURDF(
    xarm_urdf_path,
    basePosition=[0, 0, 0],  # Adjust as needed so the arm is near the tray
    useFixedBase=True        # Usually a robot arm is fixed at the base
)


# Create objects to pick
obj_half_extents = [0.02, 0.02, 0.02]  # 4cm cube
obj_collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_BOX, 
    halfExtents=obj_half_extents
)
obj_visual_shape = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=obj_half_extents,
    rgbaColor=[1, 0, 0, 1]
)


box_start_position = [0.5, 0, 0.1]  # Slightly above the tray
box_id = p.createMultiBody(
    baseMass=0.1,  # nonzero so it’s dynamic
    baseCollisionShapeIndex=obj_collision_shape,
    baseVisualShapeIndex=obj_visual_shape,
    basePosition=box_start_position
)



# Inspect link states
# for i in range(p.getNumJoints(xarm_id)):
#     info = p.getJointInfo(xarm_id, i)
#     print(i, info[12])  

grasp_object(box_id, xarm_id)

# Keep simulation running
while True:
    p.stepSimulation()
    time.sleep(1./240.)

# while True:
#     p.stepSimulation()
    
#     overhead_img = capture_image(cam_overhead)
#     # overhead_img is a tuple: (width, height, rgbData, depthData, segData, ...)
#     # Convert RGBA -> BGR for OpenCV display
#     overhead_w = overhead_img[0]
#     overhead_h = overhead_img[1]
#     overhead_rgba = overhead_img[2]  # raw data
#     overhead_rgba = np.reshape(overhead_rgba, (overhead_h, overhead_w, 4))
#     overhead_bgr = overhead_rgba[..., :3][..., ::-1]  # strip alpha, flip R->B

#     # Capture side camera
#     side_img = capture_image(cam_side)
#     side_w = side_img[0]
#     side_h = side_img[1]
#     side_rgba = side_img[2]
#     side_rgba = np.reshape(side_rgba, (side_h, side_w, 4))
#     side_bgr = side_rgba[..., :3][..., ::-1]

#     # Display each camera in its own window
#     cv2.imshow("Overhead Camera", overhead_bgr)
#     cv2.imshow("Side Camera", side_bgr)

#     # Important: allow OpenCV GUI to update
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

#     time.sleep(1/240.0)

# # Clean up
# cv2.destroyAllWindows()
# p.disconnect()

