import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

"""
How to use:

# STEP 1: Initialize the Occupancy Grid Processor
grid_processor = OccupancyGridProcessor(grid_spacing=0.1)

# STEP 2: Sample point cloud data (X, Y, Z)
points = np.array([
    [1.0, 2.0, 0.5],
    [1.2, 2.1, 0.4],
    [5.0, 8.0, 0.5],
    [5.1, 8.1, 0.6],
    [9.0, 1.0, 0.2]
])

# STEP 3: Generate Cartesian Occupancy Grid
cartesian_grid = grid_processor.calculate_cartesian_grid(points, x_limits=(-10, 10), y_limits=(0, 15))

# STEP 4: Visualize the Cartesian Occupancy Grid
plt.imshow(cartesian_grid.T, cmap=grid_processor.cmap, norm=grid_processor.norm, origin='lower')
plt.colorbar()
plt.title("Cartesian Occupancy Grid")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.show()

"""


# CLASS: OccupancyGridProcessor
class OccupancyGridProcessor:

    # FUNCTION: Initialize the grid processor with grid spacing
    def __init__(self, grid_spacing=0.5):
        # Default spacing between grid cells
        self.grid_spacing = grid_spacing  
        # Auto-initialize colormap
        self.cmap, self.norm = self.create_custom_colormap()  

    # FUNCTION: Calculate Cartesian Occupancy Grid
    def calculate_cartesian_grid(self, points, x_limits, y_limits):
        # STEP 1: Calculate grid size
        x_bins = int((x_limits[1] - x_limits[0]) / self.grid_spacing)
        y_bins = int((y_limits[1] - y_limits[0]) / self.grid_spacing)

        # STEP 2: Initialize the grid
        occupancy_grid = np.zeros((x_bins, y_bins))

        # STEP 3: Populate the grid with point counts
        for point in points:
            if len(point) == 3:
                x, y, _ = point  # Use X, Y only
            elif len(point) == 2:
                x, y = point
            else:
                raise ValueError(f"Unsupported point format: {point}")

            # STEP 4: Map points to grid cells
            if x_limits[0] <= x < x_limits[1] and y_limits[0] <= y < y_limits[1]:
                x_idx = int((x - x_limits[0]) / self.grid_spacing)
                y_idx = int((y - y_limits[0]) / self.grid_spacing)
                occupancy_grid[x_idx, y_idx] += 1  # Increment cell count

        return occupancy_grid

    # FUNCTION: Calculate Polar Occupancy Grid
    def calculate_polar_grid(self, points, range_max, range_bins, angle_bins):
        # STEP 1: Initialize the polar grid
        polar_grid = np.zeros((range_bins, angle_bins))

        # STEP 2: Populate the polar grid
        for point in points:
            if len(point) == 3:
                x, y, _ = point
            elif len(point) == 2:
                x, y = point
            else:
                raise ValueError(f"Unsupported point format: {point}")

            # STEP 3: Convert to polar coordinates
            offset = 90  # Rotate 'n' degrees clockwise
            r = np.sqrt(x**2 + y**2)  # Range
            theta = (np.degrees(np.arctan2(y, x)) + offset) % 360  # Azimuth

            # STEP 4: Map to bins
            if r < range_max:
                r_bin = int(r / (range_max / range_bins))
                theta_bin = int(theta / (360 / angle_bins))
                polar_grid[r_bin, theta_bin] += 1  # Increment cell count

        return polar_grid

    # FUNCTION: Create a Custom Colormap for Visualization
    def create_custom_colormap(self):
        # STEP 1: Define colors for different densities
        colors = [
            "white",      # Background (0 density)
            "#d1e5f0",    # Light blue (low density)
            "#92c5de",    # Blue
            "#4393c3",    # Medium density
            "#2166ac",    # High density
            "#053061"     # Very high density
        ]

        # STEP 2: Create the colormap
        cmap = ListedColormap(colors)

        # STEP 3: Define boundaries for density levels
        boundaries = [0, 1, 2, 3, 4, 5, np.inf]
        norm = BoundaryNorm(boundaries, cmap.N, clip=True)

        return cmap, norm
