import numpy as np
import matplotlib.pyplot as plt
from occupancyGrid import OccupancyGridProcessor  # Use your updated module

# Simulation Parameters
num_points = 200
x_limits = (-10, 10)
y_limits = (0, 15)
grid_spacing = 0.5

# Generate some random radar points (simulate hits)
np.random.seed(42)
random_points = np.column_stack((
    np.random.uniform(x_limits[0], x_limits[1], num_points),
    np.random.uniform(y_limits[0], y_limits[1], num_points),
    np.random.uniform(0.1, 1.0, num_points)  # Z coordinate (not used)
))

# Initialize the grid processor with Bayesian update parameters.
grid_processor = OccupancyGridProcessor(
    grid_spacing=grid_spacing, 
    p_hit=0.7,   # Increase certainty on hit
    p_miss=0.4   # (Unused in this basic version; reserved for free space updates)
)

# Update the probabilistic grid using the random points.
probability_grid = grid_processor.update_probabilistic_grid(random_points, x_limits, y_limits)

# Plot the occupancy probabilities.
plt.figure(figsize=(10, 6))
im = plt.imshow(probability_grid.T,
                cmap=grid_processor.cmap,
                norm=grid_processor.norm,
                origin='lower',
                extent=(x_limits[0], x_limits[1], y_limits[0], y_limits[1]))
cbar = plt.colorbar(im)
cbar.set_label("Occupancy Probability")
plt.title("Probabilistic Occupancy Grid (Bayesian Log-Odds Update)")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.show()
