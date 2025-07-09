import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

"""
OccupancyGridProcessor
-------------------------------
PURPOSE:
 - Generate occupancy grids using Bayesian log-odds filtering.
 - Supports initialization, updating, and probability conversion.
"""

# -------------------------------------------------------------
# CLASS: OccupancyGridProcessor
# -------------------------------------------------------------
class OccupancyGridProcessor:
    def __init__(self, grid_spacing=0.5, p_hit=0.7, p_miss=0.4):
        """
        Initialize the processor with grid spacing and log-odds parameters.
        """
        self.grid_spacing = grid_spacing  # Grid cell size
        self.p_hit = p_hit                # Probability of detection
        self.p_miss = p_miss              # Probability of miss

        self.L_hit = np.log(self.p_hit / (1 - self.p_hit))   # Log-odds increment for hit
        self.L_miss = np.log(self.p_miss / (1 - self.p_miss)) # Log-odds decrement for miss
        self.L_min = -4.6  # Minimum log-odds (≈ P = 0.01)
        self.L_max = 4.6   # Maximum log-odds (≈ P = 0.99)

        self.log_odds_grid = None         # Log-odds storage

        self.x_limits = (-10, 10)         # Default X-axis limits
        self.y_limits = (0, 15)           # Default Y-axis limits

        self.cmap, self.norm = self.create_custom_colormap() # Colormap for visualization

    # -------------------------------------------------------------
    # FUNCTION: Initialize the log-odds grid dimensions
    # -------------------------------------------------------------
    def initialize_log_odds_grid(self):
        """
        Create an empty log-odds grid based on configured limits.
        """
        x_bins = int((self.x_limits[1] - self.x_limits[0]) / self.grid_spacing)
        y_bins = int((self.y_limits[1] - self.y_limits[0]) / self.grid_spacing)
        self.log_odds_grid = np.zeros((x_bins, y_bins))

    # -------------------------------------------------------------
    # FUNCTION: Update log-odds grid using new point cloud data
    # -------------------------------------------------------------
    def update_log_odds_grid(self, points):
        """
        Apply log-odds updates for hit and miss evidence.
        """
        if self.log_odds_grid is None:
            self.initialize_log_odds_grid()

        hit_mask = np.zeros_like(self.log_odds_grid, dtype=bool)

        # Process each point in the input list
        for point in points:
            x, y = point[0], point[1]
            if self.x_limits[0] <= x < self.x_limits[1] and self.y_limits[0] <= y < self.y_limits[1]:
                x_idx = int((x - self.x_limits[0]) / self.grid_spacing)
                y_idx = int((y - self.y_limits[0]) / self.grid_spacing)
                self.log_odds_grid[x_idx, y_idx] += self.L_hit
                hit_mask[x_idx, y_idx] = True

        # Decrease log-odds for cells not hit in this frame
        miss_mask = ~hit_mask
        self.log_odds_grid[miss_mask] += self.L_miss

        # Clamp to prevent runaway values
        self.log_odds_grid = np.clip(self.log_odds_grid, self.L_min, self.L_max)

    # -------------------------------------------------------------
    # FUNCTION: Convert log-odds grid to occupancy probabilities
    # -------------------------------------------------------------
    def get_occupancy_probability_grid(self):
        """
        Calculate final occupancy probabilities.
        """
        odds = np.exp(self.log_odds_grid)
        return odds / (1 + odds)

    # -------------------------------------------------------------
    # FUNCTION: Custom colormap for visual representation
    # -------------------------------------------------------------
    def create_custom_colormap(self):
        """
        Define the color levels and boundaries.
        """
        colors = ["white", "#d1e5f0", "#92c5de", "#4393c3", "#2166ac", "#053061"]
        cmap = ListedColormap(colors)
        boundaries = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
        norm = BoundaryNorm(boundaries, cmap.N, clip=True)
        return cmap, norm
