import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

"""
Usage Example (Probabilistic):
---------------------------------
# STEP 1: Initialize the Occupancy Grid Processor with Bayesian parameters
grid_processor = OccupancyGridProcessor(grid_spacing=0.1, p_hit=0.7, p_miss=0.4)

# STEP 2: Generate or load point cloud data (X, Y, Z)
# Example: random points
points = np.array([
    [1.0, 2.0, 0.5],
    [1.2, 2.1, 0.4],
    [5.0, 8.0, 0.5],
    [5.1, 8.1, 0.6],
    [9.0, 1.0, 0.2]
])

# STEP 3: Update the probabilistic occupancy grid using Bayesian update.
x_limits = (-10, 10)
y_limits = (0, 15)
prob_grid = grid_processor.update_probabilistic_grid(points, x_limits, y_limits)

# STEP 4: Visualize the probability grid.
import matplotlib.pyplot as plt
plt.imshow(prob_grid.T, cmap=grid_processor.cmap, norm=grid_processor.norm,
           origin='lower', extent=(x_limits[0], x_limits[1], y_limits[0], y_limits[1]))
plt.colorbar(label="Occupancy Probability")
plt.title("Probabilistic Occupancy Grid (Bayesian Update)")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.show()
"""

class OccupancyGridProcessor:
    def __init__(self, grid_spacing=0.5, p_hit=0.7, p_miss=0.4, L_min=-5.0, L_max=5.0, initial_prob=0.5):
        """
        Parameters:
          grid_spacing: spacing between grid cells.
          p_hit: probability for a cell being occupied when a hit is detected.
          p_miss: probability for a cell being free when a miss is detected (if used).
          L_min, L_max: clamping limits for the log-odds values.
          initial_prob: initial occupancy probability (usually 0.5).
        """
        self.grid_spacing = grid_spacing  
        self.initial_prob = initial_prob
        self.L0 = np.log(initial_prob / (1 - initial_prob))  # Equals 0 if initial_prob is 0.5
        self.p_hit = p_hit
        self.p_miss = p_miss
        self.L_hit = np.log(p_hit / (1 - p_hit))
        self.L_miss = np.log(p_miss / (1 - p_miss))
        self.L_min = L_min
        self.L_max = L_max
        self.log_odds_grid = None  # Initialized when grid limits are provided

        # Initialize custom colormap
        self.cmap, self.norm = self.create_custom_colormap()

    def create_custom_colormap(self):
        # Define colors for occupancy probabilities.
        # Note: for a probability grid, values are in [0, 1]. So we set boundaries accordingly.
        colors = [
            "white",      # free / low occupancy
            "#d1e5f0",    # very low occupancy
            "#92c5de",    # low occupancy
            "#4393c3",    # medium occupancy
            "#2166ac",    # high occupancy
            "#053061"     # very high occupancy (close to 1)
        ]
        cmap = ListedColormap(colors)
        # Use boundaries that span [0, 1]. We use 1.01 as the upper bound (instead of np.inf) so that values of 1 are mapped correctly.
        boundaries = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.01]
        norm = BoundaryNorm(boundaries, cmap.N, clip=True)
        return cmap, norm

    def initialize_log_odds_grid(self, x_limits, y_limits):
        """
        Initialize the log-odds grid with the initial log-odds (L0) in each cell.
        """
        x_bins = int((x_limits[1] - x_limits[0]) / self.grid_spacing)
        y_bins = int((y_limits[1] - y_limits[0]) / self.grid_spacing)
        self.log_odds_grid = np.full((x_bins, y_bins), self.L0)

    def update_probabilistic_grid(self, points, x_limits, y_limits):
        """
        Update the occupancy grid using Bayesian update (log-odds formulation).
        Only cells where a hit (point) is detected are updated with L_hit.
        (In a full implementation, you could also update free space along the sensor ray using L_miss.)
        
        Parameters:
          points: An array of points. Accepts [X, Y] or [X, Y, Z] (Z is ignored).
          x_limits: Tuple (min, max) for the X-axis.
          y_limits: Tuple (min, max) for the Y-axis.
          
        Returns:
          A probability grid obtained by converting the log-odds grid.
        """
        if self.log_odds_grid is None:
            self.initialize_log_odds_grid(x_limits, y_limits)
        
        # Update log-odds grid based on the points (hits).
        for point in points:
            # Extract X, Y (ignore Z if present)
            if len(point) == 3:
                x, y, _ = point
            elif len(point) == 2:
                x, y = point
            else:
                raise ValueError(f"Unsupported point format: {point}")
            
            # Map the point to a grid cell if within limits
            if x_limits[0] <= x < x_limits[1] and y_limits[0] <= y < y_limits[1]:
                x_idx = int((x - x_limits[0]) / self.grid_spacing)
                y_idx = int((y - y_limits[0]) / self.grid_spacing)
                # Increment the log-odds with L_hit for a hit.
                self.log_odds_grid[x_idx, y_idx] += self.L_hit
                # Clamp the log-odds to avoid overflow or extreme certainty.
                self.log_odds_grid[x_idx, y_idx] = min(max(self.log_odds_grid[x_idx, y_idx], self.L_min), self.L_max)
        
        return self.get_probabilistic_grid()

    def get_probabilistic_grid(self):
        """
        Convert the current log-odds grid to an occupancy probability grid.
        Formula: P = 1 / (1 + exp(-L))
        """
        probability_grid = 1 / (1 + np.exp(-self.log_odds_grid))
        return probability_grid

    # The original count-based occupancy grid method remains available.
    def calculate_cartesian_grid(self, points, x_limits, y_limits):
        x_bins = int((x_limits[1] - x_limits[0]) / self.grid_spacing)
        y_bins = int((y_limits[1] - y_limits[0]) / self.grid_spacing)
        occupancy_grid = np.zeros((x_bins, y_bins))
        for point in points:
            if len(point) == 3:
                x, y, _ = point
            elif len(point) == 2:
                x, y = point
            else:
                raise ValueError(f"Unsupported point format: {point}")
            if x_limits[0] <= x < x_limits[1] and y_limits[0] <= y < y_limits[1]:
                x_idx = int((x - x_limits[0]) / self.grid_spacing)
                y_idx = int((y - y_limits[0]) / self.grid_spacing)
                occupancy_grid[x_idx, y_idx] += 1
        return occupancy_grid
