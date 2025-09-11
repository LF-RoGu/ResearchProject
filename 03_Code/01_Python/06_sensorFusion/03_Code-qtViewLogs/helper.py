import numpy as np
import math

# --- Sensor A Configuration ---
# Mounted facing 180°, tilted +40° → currently at 150°
# Compensate +40° to bring it back to 180°
ROTATION_DEG_A = 30
ROTATION_RAD_A = math.radians(ROTATION_DEG_A)
COS_THETA_A = math.cos(ROTATION_RAD_A)
SIN_THETA_A = math.sin(ROTATION_RAD_A)

# --- Sensor B Configuration ---
# Mounted facing 0°, tilted +40° → currently at 40°
# Compensate -40° to bring it back to 0°
ROTATION_DEG_B = -30
ROTATION_RAD_B = math.radians(ROTATION_DEG_B)
COS_THETA_B = math.cos(ROTATION_RAD_B)
SIN_THETA_B = math.sin(ROTATION_RAD_B)

PITCH_DEG = -15  # Compensate upward tilt by rotating downward
PITCH_RAD = math.radians(PITCH_DEG)
COS_PITCH = math.cos(PITCH_RAD)
SIN_PITCH = math.sin(PITCH_RAD)

def rotate_point_A(x, y):
    x_new = COS_THETA_A * x - SIN_THETA_A * y
    y_new = SIN_THETA_A * x + COS_THETA_A * y
    return x_new, y_new

def rotate_point_B(x, y):
    x_new = COS_THETA_B * x - SIN_THETA_B * y
    y_new = SIN_THETA_B * x + COS_THETA_B * y
    return x_new, y_new

def compensate_pitch(x, y, z):
    # Rotate around the X-axis: affects y (forward) and z (upward)
    y_new = COS_PITCH * y - SIN_PITCH * z
    z_new = SIN_PITCH * y + COS_PITCH * z
    return x, y_new, z_new




# -------------------------------------------------------------
# FUNCTION: average_imu_records
# PURPOSE: Average numeric fields of a list of ImuRecord objects.
# -------------------------------------------------------------
def average_imu_records(imu_records):
    if not imu_records:
        return None
    numeric_fields = [f for f, v in vars(imu_records[0]).items()
                      if isinstance(v, (int, float))]
    return {f: np.mean([getattr(r, f) for r in imu_records])
            for f in numeric_fields}


def normalize_icp_heading(rotation_avg_rad):
    """
    Converts ICP's rotation angle into IMU-compatible heading (Y-forward, CCW+).
    Assumes original ICP θ is relative rotation in robot frame.

    Adjustments:
    - Flip rotation direction
    - Shift axes: make 0 point to Y-axis (forward)

    Result is normalized to [-π, π]
    """
    # Flip direction and rotate -90° to align with Y-forward convention
    heading = np.pi - rotation_avg_rad - (np.pi / 2)
    heading = (heading + np.pi) % (2 * np.pi) - np.pi  # normalize to [-π, π]
    return heading