import numpy as np
import matplotlib.pyplot as plt
from occupancyGrid import OccupancyGridProcessor  # Use your updated module

# Simulation Parameters
num_points = 200
x_limits = (-10, 10)
y_limits = (0, 15)

# Generate random radar points (simulate hits)
np.random.seed(42)
random_points = np.column_stack((
    np.random.uniform(x_limits[0], x_limits[1], num_points),
    np.random.uniform(y_limits[0], y_limits[1], num_points),
    np.random.uniform(0.1, 1.0, num_points)  # Z coordinate (unused)
))

# -----------------------------------------------------------------------------
# 1. Fine Grid (0.5 m spacing) using the probabilistic (log‑odds) update
# -----------------------------------------------------------------------------
fine_processor = OccupancyGridProcessor(grid_spacing=0.5, p_hit=0.7, p_miss=0.4)
fine_prob_grid = fine_processor.update_probabilistic_grid(random_points, x_limits, y_limits)

# -----------------------------------------------------------------------------
# 2. Coarse Grid (2.0 m spacing) as a submap to detect objects
# -----------------------------------------------------------------------------
coarse_processor = OccupancyGridProcessor(grid_spacing=1.0, p_hit=0.7, p_miss=0.4)
coarse_prob_grid = coarse_processor.update_probabilistic_grid(random_points, x_limits, y_limits)

# Define a threshold for object detection:
object_threshold = 0.6
# Create a boolean mask for cells with occupancy above threshold.
object_cells = (coarse_prob_grid >= object_threshold)

# -----------------------------------------------------------------------------
# 3. Plotting both grids side by side
# -----------------------------------------------------------------------------
fig, ax = plt.subplots(1, 2, figsize=(14, 6))

# Plot Fine Grid
im0 = ax[0].imshow(fine_prob_grid.T, cmap=fine_processor.cmap,
                   norm=fine_processor.norm, origin='lower',
                   extent=(x_limits[0], x_limits[1], y_limits[0], y_limits[1]))
ax[0].set_title("Fine Occupancy Grid (0.5 m spacing)")
ax[0].set_xlabel("X (m)")
ax[0].set_ylabel("Y (m)")
fig.colorbar(im0, ax=ax[0], label="Occupancy Probability")

# Plot Coarse Grid (Submap)
im1 = ax[1].imshow(coarse_prob_grid.T, cmap=coarse_processor.cmap,
                   norm=coarse_processor.norm, origin='lower',
                   extent=(x_limits[0], x_limits[1], y_limits[0], y_limits[1]))
ax[1].set_title("Coarse Submap (2.0 m spacing)")
ax[1].set_xlabel("X (m)")
ax[1].set_ylabel("Y (m)")
fig.colorbar(im1, ax=ax[1], label="Occupancy Probability")

# -----------------------------------------------------------------------------
# 4. Overlay detected objects on the coarse grid
# -----------------------------------------------------------------------------
# Compute grid cell centers for the coarse grid.
x_bins = coarse_prob_grid.shape[0]
y_bins = coarse_prob_grid.shape[1]
x_edges = np.linspace(x_limits[0], x_limits[1], x_bins + 1)
y_edges = np.linspace(y_limits[0], y_limits[1], y_bins + 1)
x_centers = (x_edges[:-1] + x_edges[1:]) / 2.0
y_centers = (y_edges[:-1] + y_edges[1:]) / 2.0

# For each cell that qualifies as an object, plot a red circle.
object_indices = np.argwhere(object_cells)
for i, j in object_indices:
    x_center = x_centers[i]
    y_center = y_centers[j]
    ax[1].plot(x_center, y_center, marker='o', markersize=12,
               markerfacecolor='none', markeredgecolor='red', markeredgewidth=2)

plt.tight_layout()
plt.show()
