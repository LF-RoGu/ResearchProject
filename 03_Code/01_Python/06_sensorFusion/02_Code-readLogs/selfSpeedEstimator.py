import numpy as np

__all__ = ['estimate_self_speed']

def estimate_self_speed(pointCloud):
    # Return zero if there are no points
    if len(pointCloud) < 1:
        return 0

    phi_radspeed = []

    for point in pointCloud:
        x = point.get("x", 0)
        y = point.get("y", 0)
        doppler = point.get("doppler", 0)
        doppler = abs(doppler)  # Use absolute value of Doppler speed

        # Avoid division by zero in arctan
        if y == 0:
            continue

        phi = np.degrees(np.arctan(x / y))

        if np.isnan(phi) or np.isinf(phi):
            continue

        phi_radspeed.append([phi, doppler])

    phi_radspeed = np.array(phi_radspeed, dtype=float)

    # Must have at least 3 unique points for a 2nd degree fit
    if phi_radspeed.shape[0] < 3:
        return 0

    if np.all(phi_radspeed[:, 0] == phi_radspeed[0, 0]):
        return 0  # All angles identical

    if np.all(phi_radspeed[:, 1] == phi_radspeed[0, 1]):
        return 0  # All speeds identical

    try:
        poly_coeff = np.polyfit(phi_radspeed[:, 0], phi_radspeed[:, 1], deg=2)
    except np.linalg.LinAlgError:
        # SVD did not converge
        return 0

    poly_model = np.poly1d(poly_coeff)
    return poly_model(0)
