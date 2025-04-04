import gym
import numpy as np
import pybullet as p
import pybullet_data
from gym import spaces
import numpy.linalg as LA



class XArm6PickPlaceEnv(gym.Env):
    def __init__(self, render=False):
        super().__init__()
        # Connect to PyBullet (GUI or DIRECT)
        self.physics_client = p.connect(p.GUI if render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load plane
        plane_id = p.loadURDF("plane.urdf")

        # Load xArm6 URDF
        self.robot_id = p.loadURDF(
            "assets_repo/xarm6_with_gripper.urdf",
            basePosition=[0,0,0],
            useFixedBase=True
        )

        # Create a table (box geometry) or load from URDF
        self.table_half_extents = [0.35, 0.35, 0.02]
        collision_table = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.table_half_extents)
        # visual_table = p.createVisualShape(p.GEOM_BOX, halfExtents=self.table_half_extents, rgbaColor=[0.8, 0.6, 0.4, 1])
        # table_id = p.createMultiBody(
        #     baseMass=0,
        #     baseCollisionShapeIndex=collision_table,
        #     baseVisualShapeIndex=visual_table,
        #     basePosition=[0.5, 0, 0.3]
        # )

        # Create a small cube object
        cube_half_extents = [0.02, 0.02, 0.02]
        collision_cube = p.createCollisionShape(p.GEOM_BOX, halfExtents=cube_half_extents)
        visual_cube = p.createVisualShape(p.GEOM_BOX, halfExtents=cube_half_extents, rgbaColor=[1, 0, 0, 1])
        self.obj_id = p.createMultiBody(
            baseMass=0.2,
            baseCollisionShapeIndex=collision_cube,
            baseVisualShapeIndex=visual_cube,
            basePosition=[0.5, 0, 0]  # on top of table
        )

        # Store separate camera matrices for each camera
        self.cam1_view = p.computeViewMatrix(
            cameraEyePosition=[0.5, 0, 1.2],
            cameraTargetPosition=[0.5, 0, 0.5],
            cameraUpVector=[0, 1, 0]
        )
        self.cam1_proj = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=3.0
        )

        self.cam2_view = p.computeViewMatrix(
            cameraEyePosition=[0.3, -0.7, 0.8],
            cameraTargetPosition=[0.5, 0, 0.5],
            cameraUpVector=[0, 0, 1]
        )
        self.cam2_proj = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=3.0
        )

        # Define observation space (12-dim proprio + 2 RGB images)
        obs_proprio_low  = np.array([-np.pi]*6 + [-5.0]*6, dtype=np.float32)
        obs_proprio_high = np.array([ np.pi]*6 + [ 5.0]*6, dtype=np.float32)
        self.proprio_obs_space = spaces.Box(low=obs_proprio_low, high=obs_proprio_high, dtype=np.float32)

        img_shape = (64, 64, 3)
        self.cam_obs_space = spaces.Box(low=0, high=255, shape=img_shape, dtype=np.uint8)

        self.observation_space = spaces.Dict({
            "proprio": self.proprio_obs_space,
            "cam1": self.cam_obs_space,
            "cam2": self.cam_obs_space
        })

        # (Optional) define an action space if needed
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

    def _get_obs(self):
        # Joint data
        joint_states = p.getJointStates(self.robot_id, range(6))
        joint_positions = np.array([s[0] for s in joint_states], dtype=np.float32)
        joint_velocities = np.array([s[1] for s in joint_states], dtype=np.float32)
        proprio = np.concatenate([joint_positions, joint_velocities])

        # Camera images (explicitly pass viewMatrix=..., projectionMatrix=...)
        w, h = 64, 64
        cam1_img = p.getCameraImage(
            w, h,
            viewMatrix=self.cam1_view,
            projectionMatrix=self.cam1_proj
        )


        # cam1[2] is the raw RGBA data, shape = (w * h * 4,) or (w, h, 4) depending on PyBullet version
        cam1_rgba = np.uint8(cam1_img[2]).reshape(h, w, 4)
        # Discard alpha channel
        cam1_rgb = cam1_rgba[..., :3]  # shape = (64, 64, 3)

        cam2_img = p.getCameraImage(
            w, h,
            viewMatrix=self.cam2_view,
            projectionMatrix=self.cam2_proj
        )
        # cam1[2] is the raw RGBA data, shape = (w * h * 4,) or (w, h, 4) depending on PyBullet version
        cam2_rgba = np.uint8(cam2_img[2]).reshape(h, w, 4)
        # Discard alpha channel
        cam2_rgb = cam2_rgba[..., :3]  # shape = (64, 64, 3)
        obs = {
            "proprio": proprio,
            "cam1": cam1_rgb,
            "cam2": cam2_rgb
        }
        return obs

    def reset(self):
            """
            Each reset: 
            - optionally reset the simulation or just reposition objects,
            - randomize the cube's position & mass & friction,
            - reset the robot joints, 
            - return fresh observation.
            """
            # Option 1: Full reset of entire simulation
            p.resetSimulation()
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)

            # Re-create plane
            plane_id = p.loadURDF("plane.urdf")

            # Re-create table
            # collision_table = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.table_half_extents)
            # visual_table = p.createVisualShape(p.GEOM_BOX, halfExtents=self.table_half_extents, rgbaColor=[0.8, 0.6, 0.4, 1])
            # self.table_id = p.createMultiBody(0, collision_table, visual_table, [0.5, 0, 0.3])
            # p.changeDynamics(self.table_id, -1, lateralFriction=0.5, restitution=0.0)

            # Re-create robot
            self.robot_id = p.loadURDF(
                "assets_repo/bullet3/examples/pybullet/gym/pybullet_data/xarm/xarm6_with_gripper.urdf",
                basePosition=[0,0,0],
                useFixedBase=True
            )

             # Add a gear constraint so that joint 11 mirrors joint 8 automatically.
            # --------------------------------------------------------------------------
            drive_joint_idx = 8
            right_outer_knuckle_idx = 11

            constraint_id = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=drive_joint_idx,     # The "drive_joint"
                childBodyUniqueId=self.robot_id,
                childLinkIndex=right_outer_knuckle_idx,  # The "right_outer_knuckle_joint"
                jointType=p.JOINT_GEAR,
                jointAxis=[1, 0, 0],  # The rotation axis
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0]
            )

            # Set gear ratio to 1.0 => both joints rotate identically.
            # (You could use -1.0 if you want them to rotate in opposite directions.)
            p.changeConstraint(constraint_id, gearRatio=-1.0, maxForce=50.0)

            

            # ---- Domain Randomization: cube position, mass, friction ----
            # Sample position
            cube_x = np.random.uniform(0.4, 0.6)
            cube_y = np.random.uniform(-0.1, 0.1)
            # Sample mass
            cube_mass = np.random.uniform(0.05, 0.3)  # e.g., 50g to 300g
            # Create the collision & visual shapes
            cube_half_extents = [0.02, 0.02, 0.02]
            collision_cube = p.createCollisionShape(p.GEOM_BOX, halfExtents=cube_half_extents)
            visual_cube = p.createVisualShape(p.GEOM_BOX, halfExtents=cube_half_extents, rgbaColor=[1, 0, 0, 1])
            self.obj_id = p.createMultiBody(
                baseMass=cube_mass,
                baseCollisionShapeIndex=collision_cube,
                baseVisualShapeIndex=visual_cube,
                basePosition=[cube_x, cube_y, 0] #0.3 + 0.02
            )
            # Randomize friction
            lateral_friction = np.random.uniform(0.3, 0.6)
            p.changeDynamics(
                self.obj_id, -1,
                lateralFriction=lateral_friction,  # e.g. 0.3~0.6
                rollingFriction=0.0,
                spinningFriction=0.0,
                restitution=0.0
            )

            # (Recreate or store camera matrices if needed)
            self.cam1_view = p.computeViewMatrix([0.5, 0, 1.2], [0.5, 0, 0.5], [0, 1, 0])
            self.cam1_proj = p.computeProjectionMatrixFOV(60, 1.0, 0.1, 3.0)
            self.cam2_view = p.computeViewMatrix([0.3, -0.7, 0.8], [0.5, 0, 0.5], [0, 0, 1])
            self.cam2_proj = p.computeProjectionMatrixFOV(60, 1.0, 0.1, 3.0)

            # Reset the robot's joints to some neutral pose
            for j in range(6):
                p.resetJointState(self.robot_id, j, 0.0, 0.0)

            return self._get_obs()

    def _compute_reward(self):
        # 1) Distance-based reaching term
        #    We want the gripper to approach the object, so being closer is better (less negative).
        
        gripper_pos = np.array(p.getLinkState(self.robot_id, self.ee_link_index)[0])
        object_pos, _ = p.getBasePositionAndOrientation(self.obj_id)
        object_pos = np.array(object_pos)

        dist_to_obj = LA.norm(gripper_pos - object_pos)
        reach_reward = -dist_to_obj  # negative distance => the smaller, the better

        # 2) Grasp bonus
        #    We give a small discrete bonus once the object is actually grasped.
        grasp_reward = 0.0
        if self._is_object_grasped():
            grasp_reward = 1.0

        # 3) Holding reward (lift)
        #    If the object is grasped AND above a threshold (e.g., 0.6m from the ground),
        #    we give a small positive reward each step.
        object_height = object_pos[2]
        height_threshold = 0.6
        hold_reward = 0.0
        if self._is_object_grasped() and object_height > height_threshold:
            hold_reward = 0.05  # small positive each timestep while holding above threshold

        # 4) Time penalty (small negative each step to encourage efficiency)
        time_penalty = -0.01

        self.sub_rewards = {
        "reach": reach_reward,
        "grasp": grasp_reward,
        "hold": hold_reward
        }

        total_reward = reach_reward + grasp_reward + hold_reward + time_penalty
        return total_reward
    
    def _is_object_grasped(self):
        """
        Returns True if the robot's gripper links are in contact with the object.
        """
        # Suppose your gripper link indices are [8, 9] for the two fingers (depends on your URDF).
        finger_links = [8, 9]

        # Check for contact between each finger link and the object
        for link_idx in finger_links:
            contact_points = p.getContactPoints(bodyA=self.robot_id, bodyB=self.obj_id, linkIndexA=link_idx)
            if len(contact_points) > 0:
                # If any finger link is in contact with the object
                return True
        return False




    def step(self, action):
        # Implement the effect of your action here
        p.stepSimulation()
        obs = self._get_obs()
        reward = 0.0
        done = False
        info = {}
        return obs, reward, done, info

    def close(self):
        p.disconnect(self.physics_client)



class XArm6PickPlaceEnv(XArm6PickPlaceEnv):  # extending the previous class
    def __init__(self, render=False):
        super().__init__(render)
        # Define action space: 4 dims (dx, dy, dz, grip)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        # Identify end-effector link index (for IK). Assuming the last link is the gripper fingertip.
        self.ee_link_index = 6  # (this index might differ depending on the URDF)

        # Max step sizes
        self.max_xyz_step = 0.1  # 5 cm
        self.gripper_closed = 0.85  # value or angle representing closed gripper
        self.gripper_open = 0.0   # value representing open gripper (e.g., 4cm opening)

    def step(self, action):
        # Clip action to valid range
        action = np.clip(action, -1.0, 1.0)
        dx, dy, dz, grip = action[0], action[1], action[2], action[3]

        # Get current end-effector pose
        ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
        current_pos = np.array(ee_state[0])  # position (x, y, z)
        current_ori = ee_state[1]           # orientation (quaternion)

        # Compute target end-effector position
        target_pos = current_pos + np.array([dx, dy, dz]) * self.max_xyz_step
        # target_pos = np.array([0.5, 0.0, 0.005])
        # (Optionally enforce workspace bounds or height limits here)

        # Use inverse kinematics to get corresponding joint angles for target end-effector pose
        # Keep orientation the same to simplify (or could be fixed to pointing downwards for grasping)
        joint_angles = p.calculateInverseKinematics(self.robot_id, self.ee_link_index, target_pos, current_ori)
        # Apply joint commands (position control to move towards target angles)
        num_arm_joints = 6
        p.setJointMotorControlArray(self.robot_id, 
                                    jointIndices=list(range(num_arm_joints)), 
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=list(joint_angles[:num_arm_joints]))

        # Handle gripper action: here, we interpret grip > 0 as close, < 0 as open
        target_grip = self.gripper_closed if grip > 0 else self.gripper_open
        # Assuming gripper has one controllable joint or a coupled joint model
        p.setJointMotorControlArray(self.robot_id, 
                                    jointIndices=[8],  # example indices for two fingers
                                    controlMode=p.POSITION_CONTROL, 
                                    targetPositions=[target_grip])

        # Step the simulation
        p.stepSimulation()

        # Compute reward and done (to be defined)
        obs = self._get_obs()
        reward = self._compute_reward()  # placeholder for reward function
        done = False  # done can be set True when task success or time limit
        info = {}
        return obs, reward, done, info
