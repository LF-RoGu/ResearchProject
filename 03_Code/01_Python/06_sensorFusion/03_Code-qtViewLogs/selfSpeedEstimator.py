import numpy as np
import math

def estimate_ego_speed_scalar(inlierPoints):
    """
    Estimate scalar ego-vehicle speed assuming forward motion (+y axis).
    Returns 0.0 if the input is invalid or insufficient.
    """
    if not inlierPoints:
        return 0.0

    theta_list = []
    doppler_list = []
    
    for pt in inlierPoints:
        if 'x' not in pt or 'y' not in pt or 'doppler' not in pt:
            continue
        # Shift theta so that 0° is forward (+y axis)
        theta = math.atan2(pt['y'], pt['x']) - math.pi/2
        theta_list.append(theta)
        doppler_list.append(pt['doppler'])
    
    if len(theta_list) < 3:
        return 0.0

    theta_arr = np.array(theta_list)
    doppler_arr = np.array(doppler_list)

    cos_theta = np.cos(theta_arr)
    denominator = np.sum(cos_theta**2)
    if denominator == 0:
        return 0.0

    v_ego = -np.sum(doppler_arr * cos_theta) / denominator
    return v_ego


def estimate_ego_velocity_vector(inlierPoints):
    """
    Estimate 2D ego-velocity vector (vx, vy) using Doppler and azimuth.
    Forward motion is along +y axis.
    """
    if not inlierPoints:
        return 0.0, 0.0

    A = []
    b = []
    
    for pt in inlierPoints:
        if 'x' not in pt or 'y' not in pt or 'doppler' not in pt:
            continue
        # Shift theta so that 0° is forward (+y axis)
        theta = math.atan2(pt['y'], pt['x']) - math.pi/2
        A.append([np.cos(theta), np.sin(theta)])
        b.append(pt['doppler'])

    if len(A) < 3:
        return 0.0, 0.0

    A = np.array(A)
    b = np.array(b).reshape(-1, 1)

    try:
        v_xy, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        vx, vy = -v_xy.flatten()
        return vx, vy
    except np.linalg.LinAlgError:
        return 0.0, 0.0
