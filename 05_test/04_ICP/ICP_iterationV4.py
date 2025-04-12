import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

def angle_from_position(x, y):
    return np.arctan2(y, x)

def solve_velocity_from_doppler_and_angle(rr1, phi1, rr2, phi2):
    A = np.array([
        [np.cos(phi1), np.sin(phi1)],
        [np.cos(phi2), np.sin(phi2)]
    ])
    b = np.array([rr1, rr2])
    try:
        vx_vy = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        vx_vy = np.array([0.0, 0.0])  # Fallback if system is singular
    return vx_vy

# Simulate centroids with ID and radial speed
np.random.seed(1)
num_points = 6

prev_centroids = []
curr_centroids = []

for i in range(num_points):
    pos_prev = np.array([np.random.uniform(-5, 5), np.random.uniform(10, 15)])
    motion = np.array([0.0, -0.5])  # Simulate forward ego-motion
    pos_curr = pos_prev + motion + np.random.normal(0, 0.02, 2)
    
    phi_prev = angle_from_position(*pos_prev)
    phi_curr = angle_from_position(*pos_curr)

    rr_prev = motion[0] * np.cos(phi_prev) + motion[1] * np.sin(phi_prev)
    rr_curr = motion[0] * np.cos(phi_curr) + motion[1] * np.sin(phi_curr)

    prev_centroids.append([*pos_prev, i, 0, rr_prev])  # x, y, ID, frame, doppler
    curr_centroids.append([*pos_curr, i, 1, rr_curr])

# Combine into a single array
all_points = np.array(prev_centroids + curr_centroids)

# Cluster using DBSCAN
clustering = DBSCAN(eps=1.0, min_samples=2).fit(all_points[:, :2])
labels = clustering.labels_
all_points = np.column_stack((all_points, labels))  # Add cluster label

# Estimate motion vectors
motion_vectors = []
for label in np.unique(labels):
    if label == -1:
        continue  # Skip noise
    cluster = all_points[labels == label]
    if cluster.shape[0] != 2:
        continue  # Only handle matched pairs

    p1, p2 = cluster
    if p1[3] == p2[3]:
        continue  # Both from same frame, skip

    # Ensure p1 is from frame 0, p2 from frame 1
    if p1[3] == 1:
        p1, p2 = p2, p1

    phi1 = angle_from_position(p1[0], p1[1])
    phi2 = angle_from_position(p2[0], p2[1])
    rr1 = p1[4]
    rr2 = p2[4]

    vx, vy = solve_velocity_from_doppler_and_angle(rr1, phi1, rr2, phi2)
    motion_vectors.append((p2[0], p2[1], vx, vy))

# Visualization
plt.figure(figsize=(8, 6))
plt.title("Motion Estimation Using Clustered RR Data")
plt.xlabel("X (Left/Right)")
plt.ylabel("Y (Forward)")
plt.grid(True)
plt.axis("equal")

for x, y, vx, vy in motion_vectors:
    plt.quiver(x, y, vx, vy, angles='xy', scale_units='xy', scale=1, color='red', width=0.005)

plt.scatter(all_points[:, 0], all_points[:, 1], c=all_points[:, -1], cmap='tab10', label='Clustered Points')
plt.colorbar(label='Cluster ID')
plt.legend()
plt.tight_layout()
plt.show()
