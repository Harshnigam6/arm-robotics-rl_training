import pybullet as p

def create_view_matrix_from_params(cam_config):
    """
    cam_config is a dict that includes:
      - cameraTargetPosition (list): [x, y, z] the point the camera looks at
      - distance (float): distance from the target
      - yaw (float): yaw angle in degrees
      - pitch (float): pitch angle in degrees
      - roll (float): roll angle in degrees
      - upAxisIndex (int): typically 2 if Z is up
    """
    return p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=cam_config['target'],
        distance=cam_config['distance'],
        yaw=cam_config['yaw'],
        pitch=cam_config['pitch'],
        roll=cam_config['roll'],
        upAxisIndex=cam_config['upAxisIndex']
    )

def create_projection_matrix_from_params(cam_config):
    """
    cam_config should also include:
      - width (int)
      - height (int)
      - fov (float): field of view in degrees
      - near (float)
      - far (float)
    """
    aspect = cam_config['width'] / cam_config['height']
    return p.computeProjectionMatrixFOV(
        fov=cam_config['fov'],
        aspect=aspect,
        nearVal=cam_config['near'],
        farVal=cam_config['far']
    )

def capture_image(cam_config):
    """
    Generates a PyBullet camera image tuple (w, h, rgb, depth, seg).
    """
    view_matrix = create_view_matrix_from_params(cam_config)
    proj_matrix = create_projection_matrix_from_params(cam_config)
    
    # p.getCameraImage returns a tuple:
    # (width, height, [RGB data], [Depth data], [Seg data], ...)
    img_data = p.getCameraImage(
        width=cam_config['width'],
        height=cam_config['height'],
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL  # or p.ER_TINY_RENDERER, etc.
    )
    return img_data
