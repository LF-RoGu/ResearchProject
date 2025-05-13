import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

# -------------------------------
# FUNCTION: Cartesian Occupancy Grid
# -------------------------------
def calculate_occupancy_grid(points, x_limits, y_limits, grid_spacing):
    """
    Calculate an occupancy grid for the given points.

    Parameters:
        points (list of tuples): List of (x, y) or (x, y, z) coordinates.
        x_limits (tuple): The x-axis limits as (xmin, xmax).
        y_limits (tuple): The y-axis limits as (ymin, ymax).
        grid_spacing (int): Spacing between grid cells.

    Returns:
        np.ndarray: 2D occupancy grid.
    """
    # Calculate grid size
    x_bins = int((x_limits[1] - x_limits[0]) / grid_spacing)
    y_bins = int((y_limits[1] - y_limits[0]) / grid_spacing)

    # Initialize the grid
    occupancy_grid = np.zeros((x_bins, y_bins))

    # Populate the grid
    for point in points:
        if len(point) == 3:
            x, y, _ = point  # Unpack x, y, z
        elif len(point) == 2:
            x, y = point  # Unpack x, y only
        else:
            raise ValueError(f"Point format not supported: {point}")

        if x_limits[0] <= x < x_limits[1] and y_limits[0] <= y < y_limits[1]:
            x_idx = int((x - x_limits[0]) / grid_spacing)
            y_idx = int((y - y_limits[0]) / grid_spacing)
            occupancy_grid[x_idx, y_idx] += 1

    return occupancy_grid

# -------------------------------
# FUNCTION: Polar Occupancy Grid
# -------------------------------
def calculate_polar_occupancy_grid(points, range_max, range_bins, angle_bins):
    """
    Calculate an occupancy grid in polar coordinates.

    Parameters:
        points (list of tuples): List of (x, y) or (x, y, z) coordinates.
        range_max (float): Maximum range for the grid.
        range_bins (int): Number of bins for range.
        angle_bins (int): Number of bins for angles.

    Returns:
        np.ndarray: 2D polar occupancy grid.
    """
    # Initialize the grid
    polar_grid = np.zeros((range_bins, angle_bins))

    # Iterate through points and fill the polar grid
    for point in points:
        if len(point) == 3:
            x, y, _ = point  # Unpack x, y, z
        elif len(point) == 2:
            x, y = point  # Unpack x, y only
        else:
            raise ValueError(f"Point format not supported: {point}")

        # Compute polar coordinates
        offset = 90  # Rotate 90 degrees clockwise (adjust as needed)
        r = np.sqrt(x**2 + y**2)
        theta = (np.degrees(np.arctan2(y, x)) + offset) % 360  # Normalize to [0, 360)

        # Map to bins
        if r < range_max:
            r_bin = int(r / (range_max / range_bins))
            theta_bin = int(theta / (360 / angle_bins))
            polar_grid[r_bin, theta_bin] += 1

    return polar_grid

def create_custom_colormap():
        """
        Create a custom colormap for the occupancy grid.
        Returns:
            cmap: Custom colormap with a specific background color.
            norm: Normalizer to map data values to colormap levels.
        """
        # Define colors: First is the background color, followed by density colors
        colors = [
            "white",      # Background color (e.g., for 0)
            "#d1e5f0",    # Light blue (low density)
            "#92c5de",    # Blue
            "#4393c3",    # Medium density
            "#2166ac",    # Dark blue (high density)
            "#053061"     # Very high density
        ]
        cmap = ListedColormap(colors)

        # Define boundaries for each color bin
        boundaries = [0, 1, 2, 3, 4, 5, np.inf]  # Bins for densities
        norm = BoundaryNorm(boundaries, cmap.N, clip=True)

        return cmap, norm