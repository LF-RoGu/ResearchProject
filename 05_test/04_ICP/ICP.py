import numpy as np
from scipy.spatial import cKDTree

def icp_translation_only(prev_points, curr_points, max_iterations=10, tolerance=1e-4):
    """
    Estimate 2D translation between two sets of centroids using ICP (no rotation).
    
    Parameters:
        prev_points: np.array of shape (N, 2) from frame t-1
        curr_points: np.array of shape (N, 2) from frame t
    Returns:
        translation: np.array([vx, vy])
        aligned_prev: transformed previous points after alignment
        residuals: distance between matched pairs after alignment
    """
    prev = prev_points.copy()
    curr = curr_points
    translation = np.zeros(2)

    for _ in range(max_iterations):
        tree = cKDTree(curr)
        distances, indices = tree.query(prev)

        matched_curr = curr[indices]
        delta = matched_curr - prev
        mean_shift = np.mean(delta, axis=0)

        prev += mean_shift
        translation += mean_shift

        if np.linalg.norm(mean_shift) < tolerance:
            break

    residuals = np.linalg.norm(curr[indices] - prev, axis=1)
    return translation, prev, residuals



# Example centroid arrays
prev_centroids = np.array([
    [2.0, 5.0],
    [3.0, 8.0],
    [1.0, 10.0]
])

curr_centroids = np.array([
    [2.1, 5.5],
    [3.1, 8.5],
    [1.1, 10.5]
])

translation, aligned_prev, residuals = icp_translation_only(prev_centroids, curr_centroids)

print(f"Ego-motion estimate: Δx = {translation[0]:.2f}, Δy = {translation[1]:.2f}")
