import numpy as np

__all__ = ['estimate_self_speed']

def estimate_self_speed(pointCloud):
    #Returning zero if there are no points to process
    if len(pointCloud) < 1:
        return 0

    #Preparing an array to contain angle to target and radial speed
    phi_radspeed = []

    #Iterating over all points
    for i in range(len(pointCloud)):
        #Calculating the angle to target
        phi = np.rad2deg(np.arctan(pointCloud[i]["x"]/pointCloud[i]["y"]))

        #Appending the angle and the radial speed 
        phi_radspeed.append([phi, pointCloud[i]["doppler"]])

    #Converting array of tuples to NumPy array
    phi_radspeed = np.array(phi_radspeed, dtype=float)

    #Fitting a first order polynominal into the points
    poly_coeff = np.polyfit(phi_radspeed[:,0], phi_radspeed[:,1], deg=2)  # Polynomial coefficients
    poly_model = np.poly1d(poly_coeff)  # Polynomial model

    #Returning the self-speed after interpolating
    return poly_model(0)