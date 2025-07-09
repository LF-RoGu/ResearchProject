"""
OccupancyGridProcessor
-------------------------------------------------------------
PURPOSE:
  This module implements an occupancy grid using a log-odds
  Bayesian filter for radar point cloud mapping.

  It provides:
   - Grid initialization
   - Log-odds updates for hits/misses
   - Conversion to occupancy probabilities
   - Custom colormap for clear visualization
-------------------------------------------------------------
"""

import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

# -------------------------------------------------------------
# CLASS: OccupancyGridProcessor
# PURPOSE: Handles grid storage, log-odds updates, and output.
# -------------------------------------------------------------
class OccupancyGridProcessor:
    def __init__(self, grid_spacing=0.5, p_hit=0.7, p_miss=0.4):
        """
        Initialize the occupancy grid processor.

        Args:
          grid_spacing (float): Cell size in meters.
          p_hit (float): Probability for a detected hit.
          p_miss (float): Probability for a miss.
        """
        self.grid_spacing = grid_spacing
        self.p_hit = p_hit
        self.p_miss = p_miss

        # Log-odds updates for hit and miss events
        self.L_hit = np.log(self.p_hit / (1 - self.p_hit))
        self.L_miss = np.log(self.p_miss / (1 - self.p_miss))

        # Clamp range to avoid numeric overflow
        self.L_min = -4.6   # logit(0.01)
        self.L_max = 4.6    # logit(0.99)

        # Initialize log-odds storage grid
        self.log_odds_grid = None

        # Default limits for local Cartesian grid
        self.x_limits = (-10, 10)
        self.y_limits = (0, 15)

        # Custom color map for visualizing occupancy
        self.cmap, self.norm = self.create_custom_colormap()

    # -------------------------------------------------------------
    # FUNCTION: Initialize empty log-odds grid
    # -------------------------------------------------------------
    def initialize_log_odds_grid(self):
        """
        Allocate grid based on X/Y limits and spacing.
        """
        x_bins = int((self.x_limits[1] - self.x_limits[0]) / self.grid_spacing)
        y_bins = int((self.y_limits[1] - self.y_limits[0]) / self.grid_spacing)
        self.log_odds_grid = np.zeros((x_bins, y_bins))

    # -------------------------------------------------------------
    # FUNCTION: Update log-odds grid given point cloud hits
    # -------------------------------------------------------------
    def update_log_odds_grid(self, points):
        """
        Update grid using hit/miss log-odds logic.

        Args:
          points (list of [x,y,z,...]): Point cloud data.
        """
        if self.log_odds_grid is None:
            self.initialize_log_odds_grid()

        hit_mask = np.zeros_like(self.log_odds_grid, dtype=bool)

        for point in points:
            x, y = point[0], point[1]

            if self.x_limits[0] <= x < self.x_limits[1] and self.y_limits[0] <= y < self.y_limits[1]:
                x_idx = int((x - self.x_limits[0]) / self.grid_spacing)
                y_idx = int((y - self.y_limits[0]) / self.grid_spacing)
                self.log_odds_grid[x_idx, y_idx] += self.L_hit
                hit_mask[x_idx, y_idx] = True

        miss_mask = ~hit_mask
        self.log_odds_grid[miss_mask] += self.L_miss

        # Clamp to bounds
        self.log_odds_grid = np.clip(self.log_odds_grid, self.L_min, self.L_max)

    # -------------------------------------------------------------
    # FUNCTION: Convert log-odds to occupancy probability
    # -------------------------------------------------------------
    def get_occupancy_probability_grid(self):
        """
        Compute probability grid: P = odds / (1 + odds)
        """
        odds = np.exp(self.log_odds_grid)
        return odds / (1 + odds)

    # -------------------------------------------------------------
    # FUNCTION: Create custom colormap for grid display
    # -------------------------------------------------------------
    def create_custom_colormap(self):
        """
        Define occupancy colors and thresholds.
        """
        colors = ["white", "#d1e5f0", "#92c5de", "#4393c3", "#2166ac", "#053061"]
        cmap = ListedColormap(colors)
        boundaries = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
        norm = BoundaryNorm(boundaries, cmap.N, clip=True)
        return cmap, norm
