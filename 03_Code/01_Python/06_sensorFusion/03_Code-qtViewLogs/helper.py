import numpy as np
import math

# --- Sensor A Configuration ---
# Mounted facing 180°, tilted +30° → currently at 150°
# Compensate +30° to bring it back to 180°
ROTATION_DEG_A = 40
ROTATION_RAD_A = math.radians(ROTATION_DEG_A)
COS_THETA_A = math.cos(ROTATION_RAD_A)
SIN_THETA_A = math.sin(ROTATION_RAD_A)

# --- Sensor B Configuration ---
# Mounted facing 0°, tilted +30° → currently at 30°
# Compensate -30° to bring it back to 0°
ROTATION_DEG_B = -40
ROTATION_RAD_B = math.radians(ROTATION_DEG_B)
COS_THETA_B = math.cos(ROTATION_RAD_B)
SIN_THETA_B = math.sin(ROTATION_RAD_B)


def rotate_point_A(x, y):
    x_new = COS_THETA_A * x - SIN_THETA_A * y
    y_new = SIN_THETA_A * x + COS_THETA_A * y
    return x_new, y_new

def rotate_point_B(x, y):
    x_new = COS_THETA_B * x - SIN_THETA_B * y
    y_new = SIN_THETA_B * x + COS_THETA_B * y
    return x_new, y_new




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