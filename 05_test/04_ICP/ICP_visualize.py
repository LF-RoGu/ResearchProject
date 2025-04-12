import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

def extract_centroids(points, eps=1.0, min_samples=1):
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_
    centroids = []
    for label in np.unique(labels):
        if label == -1:
            continue  # Skip noise
        cluster_points = points[labels == label]
        centroid = np.mean(cluster_points, axis=0)
        centroids.append(centroid)
    return np.array(centroids)

def match_centroids(prev_centroids, curr_centroids, max_distance=1.0):
    matches = []
    for c in curr_centroids:
        distances = np.linalg.norm(prev_centroids - c, axis=1)
        min_idx = np.argmin(distances)
        if distances[min_idx] < max_distance:
            matches.append((c, prev_centroids[min_idx]))
    return matches

def estimate_translation_from_matches(matches):
    if not matches:
        return np.zeros(2), [], []
    deltas = [prev - curr for curr, prev in matches]
    mean_shift = np.mean(deltas, axis=0)
    aligned_curr = [curr + mean_shift for curr, _ in matches]
    matched_prev = [prev for _, prev in matches]
    residuals = np.linalg.norm(np.array(matched_prev) - np.array(aligned_curr), axis=1)
    return -mean_shift, np.array(aligned_curr), np.array(matched_prev), residuals

# Simulate raw point clouds
np.random.seed(42)
num_objects = 20
prev_points = np.column_stack((
    np.random.uniform(-5, 5, size=num_objects),
    np.random.uniform(10, 15, size=num_objects)
))

vehicle_translation = np.array([0.0, -0.5])
curr_points = prev_points + vehicle_translation + np.random.normal(0, 0.05, prev_points.shape)

# Extract centroids via DBClustering
prev_centroids = extract_centroids(prev_points)
curr_centroids = extract_centroids(curr_points)

# Match and estimate motion
matches = match_centroids(prev_centroids, curr_centroids)
estimated_translation, aligned_curr, matched_prev, residuals = estimate_translation_from_matches(matches)

# Plotting
plt.figure(figsize=(8, 6))
plt.title("Object Direction via DBSCAN + Matching")
plt.xlabel("X (Left/Right)")
plt.ylabel("Y (Forward)")
plt.grid(True)
plt.axis("equal")

plt.scatter(prev_centroids[:, 0], prev_centroids[:, 1], color='blue', label='Prev Centroids')
plt.scatter(curr_centroids[:, 0], curr_centroids[:, 1], color='green', label='Curr Centroids')
plt.scatter(aligned_curr[:, 0], aligned_curr[:, 1], color='cyan', marker='x', label='Aligned Curr')

for a, b in zip(aligned_curr, matched_prev):
    plt.plot([a[0], b[0]], [a[1], b[1]], 'r--', linewidth=1)

origin = np.mean(curr_centroids, axis=0)
plt.quiver(origin[0], origin[1], estimated_translation[0], estimated_translation[1],
           angles='xy', scale_units='xy', scale=1, color='orange', width=0.005, label='Estimated Object Motion')

plt.legend()
plt.tight_layout()
plt.show()
