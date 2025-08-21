import numpy as np
import math
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

__all__ = ['filterSNR', 'filterCartesianX', 'filterCartesianY', 'filterCartesianZ', 'filterSphericalR', 'filterSphericalTheta', 'filterSphericalPhi', 'filter_by_speed']

def filterSNRmin(inputPoints, snr_min):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_x = inputPoints[i]["snr"]
            if point_x >= snr_min:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
        print(f"Error filtering points: {e}")
        return None
    return filteredPoints

def filterCartesianX(inputPoints, x_min, x_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_x = inputPoints[i]["x"]
            if point_x >= x_min and point_x <= x_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
        print(f"Error filtering points: {e}")
        return None
    return filteredPoints

def filterCartesianY(inputPoints, y_min, y_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_x = inputPoints[i]["x"]
            if point_x >= y_min and point_x <= y_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
        print(f"Error filtering points: {e}")
        return None
    return filteredPoints

def filterCartesianY(inputPoints, y_min, y_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_y = inputPoints[i]["y"]
            if point_y >= y_min and point_y <= y_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
        print(f"Error filtering points: {e}")
        return None
    return filteredPoints

def filterCartesianZ(inputPoints, z_min, z_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_z = inputPoints[i]["z"]
            if point_z >= z_min and point_z <= z_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
        print(f"Error filtering points: {e}")
        return None
    return filteredPoints

def filter_moving_objects_ransac(pointCloud, return_model=False):
    """
    Detects moving objects using RANSAC fit of Doppler vs azimuth angle.
    
    Parameters:
        pointCloud: list of dicts with 'x', 'y', and 'doppler' keys

    Returns:
        List of points considered to be moving objects (i.e., outliers to the static Doppler model).
    """
    if not pointCloud:
        if return_model:
            return {
                "model": None,
                "inliers": [],
                "outliers": [],
                "X": np.empty((0, 1)),
                "y": np.array([]),
                "theta": [],
                "doppler": []
            }
        return []

    try:
        theta_list = []
        doppler_list = []

        for pt in pointCloud:
            if 'x' in pt and 'y' in pt and 'doppler' in pt:
                theta = math.atan2(pt['y'], pt['x'])  # azimuth in vehicle coords
                theta -= math.pi / 2                  # rotate so forward (+Y) is 0 rad
                # Wrap to [-pi, pi] to keep RANSAC happy
                if theta > math.pi:
                    theta -= 2 * math.pi
                elif theta < -math.pi:
                    theta += 2 * math.pi

                theta_list.append(theta)
                doppler_list.append(pt['doppler'])

        if len(theta_list) < 2:
            # Not enough points to reliably fit a model
            if return_model:
                return {
                    "model": None,
                    "inliers": [],
                    "outliers": [],
                    "X": np.empty((0, 1)),
                    "y": np.array([]),
                    "theta": theta_list,
                    "doppler": doppler_list
                }
            # No reliable model → no outliers detected
            return []

        X = np.array(theta_list).reshape(-1, 1)
        y = np.array(doppler_list)

        # RANSAC model for 2nd-degree polynomial
        model = make_pipeline(
            PolynomialFeatures(degree=2),
            RANSACRegressor(min_samples=0.3, residual_threshold=1.5)
        )
        model.fit(X, y)

        # Get mask from RANSAC model
        inlier_mask = model.named_steps['ransacregressor'].inlier_mask_

        # Identify outliers (moving points)
        inlier_points = [pt for i, pt in enumerate(pointCloud) if inlier_mask[i]]
        moving_points = [pt for i, pt in enumerate(pointCloud) if not inlier_mask[i]]

        if return_model:
            return {
                "model": model,
                "inliers": inlier_points,
                "outliers": moving_points,
                "X": X,
                "y": y,
                "theta": theta_list,
                "doppler": doppler_list
            }
        else:
            return moving_points


    except Exception as e:
        print(f"[Error] RANSAC filtering failed: {e}")
        if return_model:
            return {
                "model": None,
                "inliers": np.array([], dtype=bool),
                "outliers": [],
                "X": np.empty((0, 1)),
                "y": np.array([]),
                "theta": [],
                "doppler": []
            }
        return []

def filterDoppler(inputPoints, doppler_min, doppler_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_doppler = inputPoints[i]["doppler"]
            if abs(point_doppler) >= doppler_min and abs(point_doppler) <= doppler_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
        print(f"Error filtering points: {e}")
        return None
    return filteredPoints

def filterSphericalR(inputPoints, r_min, r_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_r = np.sqrt(inputPoints[i]["x"]**2 + inputPoints[i]["y"]**2 + inputPoints[i]["z"]**2)
            if point_r >= r_min and point_r <= r_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
                print(f"Error filtering points: {e}")
                return None
    return filteredPoints

def filterSphericalTheta(inputPoints, theta_min, theta_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_r = np.sqrt(inputPoints[i]["x"]**2 + inputPoints[i]["y"]**2 + inputPoints[i]["z"]**2)
            point_theta = np.rad2deg(np.arccos(inputPoints[i]["z"] / point_r))
            if point_theta >= theta_min and point_theta <= theta_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
                print(f"Error filtering points: {e}")
                return None
    return filteredPoints

def filterSphericalPhi(inputPoints, phi_min, phi_max):
    filteredPoints = []
    try:
        for i in range(len(inputPoints)):
            point_phi = np.rad2deg(np.arctan(inputPoints[i]["x"]/inputPoints[i]["y"]))
            
            if point_phi >= phi_min and point_phi <= phi_max:
                filteredPoints.append(inputPoints[i])
    except (ValueError, IndexError) as e:
                print(f"Error filtering points: {e}")
                return None
    return filteredPoints

def filter_by_speed(inputPoints, self_speed, speed_threshold):
    filteredPoints = []
    try:
        # STEP 1: Calculate allowable Doppler speed range
        lower_bound = self_speed - speed_threshold
        upper_bound = self_speed + speed_threshold

        # STEP 2: Filter points within the Doppler speed threshold
        for point in inputPoints:
            doppler = point['doppler']
            if (lower_bound <= doppler <= upper_bound) and (doppler != 0):
                filteredPoints.append(point)

    except (ValueError, IndexError, KeyError) as e:
        print(f"Error filtering points by speed percentile: {e}")
        return None

    return filteredPoints

# -------------------------------
# FUNCTION: Extract points from any dictionary
# -------------------------------
def extract_points(data):
    """
    Extract points from various dictionary formats and convert to NumPy array.

    Args:
        data (list/dict or np.ndarray): Input data with points.

    Returns:
        np.ndarray: 2D array with [x, y, z] columns.
    """
    if not data or len(data) == 0:
        return np.empty((0, 3))

    if isinstance(data, list):
        if isinstance(data[0], dict):
            # Look for 'x', 'y', 'z' keys in any dictionary
            return np.array([[item.get("x", 0), item.get("y", 0), item.get("z", 0), item.get("doppler", 0)] for item in data])
        else:
            return np.array(data)
    elif isinstance(data, dict):
        # If data is a dictionary with 'detectedPoints'
        if 'detectedPoints' in data:
            return np.array([[point.get("x", 0), point.get("y", 0), point.get("z", 0)] for point in data['detectedPoints']])
        # If data is clustered, extract 'points' from clusters
        elif all(isinstance(value, dict) and 'points' in value for value in data.values()):
            return np.vstack([value['points'] for value in data.values()])
    elif isinstance(data, np.ndarray):
        return data if data.ndim == 2 else data.reshape(-1, 3)

    raise ValueError("Unsupported data format for clustering.")