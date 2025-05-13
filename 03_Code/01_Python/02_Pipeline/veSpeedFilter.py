import numpy as np

__all__ = ['calculateVe']

def calculateVe(point_cloud):
    point_cloud_ve = []
    
    #Iterating over all points for calculaing ve
    for i in range(len(point_cloud)):
        point = point_cloud[i]
        
        #Calculating phi
        phi = np.rad2deg(np.arctan(point["x"]/point["y"]))
        
        #Calculating ve
        point["ve"] = point["doppler"] / np.cos(np.deg2rad(phi))

        point_cloud_ve.append(point)

    return point_cloud_ve


def filterPointsWithVe(point_cloud, self_speed_filtered):
    point_cloud_ve_filtered = []

    ##Filtering using IQR method

    #Creating a vector with all values of ve
    ve = np.array([point["ve"] for point in point_cloud])
    
    #Calculating Q1 (25th percentile) and Q3 (75th percentile)
    Q1 = np.percentile(ve, 25)
    Q3 = np.percentile(ve, 75)
    
    #Calculating the IQR
    IQR = Q3 - Q1

    #Defining the bounds for outliers
    ve_lower_bound = Q1 - 1.5 * IQR
    ve_upper_bound = Q3 + 1.5 * IQR

    #Iterating over the list and adding only the points to the return list that are no outliers
    for i in range(len(point_cloud)):
        if point_cloud[i]["ve"] >= ve_lower_bound and point_cloud[i]["ve"] <= ve_upper_bound:
            point_cloud_ve_filtered.append(point_cloud[i])

    return point_cloud_ve_filtered