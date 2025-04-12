import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from matplotlib.patches import Rectangle, Ellipse

# -------------------------------
# FUNCTION: Cluster Points
# -------------------------------
def cluster_points(points, eps=1.0, min_samples=6):
    """ Perform DBSCAN clustering and filter clusters based on priorities. """
    dbscan = DBSCAN(eps=eps, min_samples=min_samples).fit(points[:, :3])  # Use X, Y, Z for clustering
    labels = dbscan.labels_

    clusters = {}
    cluster_range_azimuth = []  # Calculate range and azimuth.
    for cluster_id in np.unique(labels):
        if cluster_id == -1:  # Ignore noise
            continue
        cluster_points = points[labels == cluster_id]
        size = len(cluster_points)

        # Ignore clusters with <3 points (Priority 3)
        if size < 3:
            continue

        # Store centroid and priority
        centroid = np.mean(cluster_points, axis=0)
        range_to_origin = np.linalg.norm(centroid[:2])  # Range from centroid to origin (X, Y)
        azimuth_to_origin = np.degrees(np.arctan2(centroid[1], centroid[0]))  # Azimuth angle

        # Save range and azimuth
        cluster_range_azimuth.append((cluster_id, range_to_origin, azimuth_to_origin, cluster_points))

        # Store centroid and priority
        centroid = np.mean(cluster_points, axis=0)
        if size >= 10:
            priority = 3
        elif size < 10 and size >= 5:
            priority = 2
        elif size < 5:
            priority = 1
        else:
            priority = 4
        clusters[cluster_id] = {'centroid': centroid, 'priority': priority, 'points': cluster_points}

    return clusters, cluster_range_azimuth

# -------------------------------
# FUNCTION: Plot Clusters
# -------------------------------
def plot_clusters(clusters, ax):
    """ Plot clusters and visualize bounding boxes and priorities. """
    for cid, cluster in clusters.items():
        centroid = cluster['centroid']
        ax.scatter(cluster['points'][:, 0], cluster['points'][:, 1], label=f"Cluster {cid}")
        ax.scatter(centroid[0], centroid[1], c='black', marker='x')  # Centroid marker

        # Draw bounding box
        width = np.max(cluster['points'][:, 0]) - np.min(cluster['points'][:, 0])
        height = np.max(cluster['points'][:, 1]) - np.min(cluster['points'][:, 1])
        ax.add_patch(Rectangle(
            (centroid[0] - width / 2, centroid[1] - height / 2), width, height,
            fill=False, edgecolor='purple', linewidth=1.5
        ))

        # Add priority labels
        ax.text(centroid[0], centroid[1], f"P{cluster['priority']}", color='red')