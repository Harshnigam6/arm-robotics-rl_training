import numpy.linalg as LA

def _compute_reward(self):
    # Get positions
    gripper_pos = np.array(p.getLinkState(self.robot_id, self.ee_link_index)[0])
    object_pos, _ = p.getBasePositionAndOrientation(self.obj_id)
    object_pos = np.array(object_pos)
    goal_pos = self.goal_position  # predefined target location for placing

    # Distance terms
    dist_to_obj = LA.norm(gripper_pos - object_pos)
    dist_to_goal = LA.norm(object_pos - goal_pos)

    # Reaching reward (closer to object is better)
    reach_reward = -dist_to_obj  
    # Grasping reward
    grasp_reward = 0.0
    if self._is_object_grasped():
        grasp_reward = 1.0
    # Lifting reward
    lift_reward = 0.0
    object_height = object_pos[2]
    if object_height > 0.6:  # if lifted above table height (assuming table at z=0.5)
        lift_reward = 0.5
    # Placing reward (only start giving when object is grasped)
    place_reward = 0.0
    if self._is_object_grasped():
        place_reward = -dist_to_goal

    # Success reward
    success_reward = 0.0
    if self._is_success():
        success_reward = 5.0

    # Time penalty
    time_penalty = -0.01

    total_reward = reach_reward + grasp_reward + lift_reward + place_reward + success_reward + time_penalty
    return total_reward
