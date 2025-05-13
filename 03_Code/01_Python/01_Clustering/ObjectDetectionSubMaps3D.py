import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from sklearn.cluster import DBSCAN
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.gridspec import GridSpec

from DataProcessing.radar_utilsProcessing import *
from DataProcessing.radar_utilsPlot import *
from OccupancyGrid.OccupancyGrid import *
from Clustering.dbClustering import *

SAFETY_BOX_CENTER = [0, 2, 0]  # Center position (X, Y, Z)
SAFETY_BOX_SIZE = [2, 9, 2]   # Width, Height, Depth

# Create a new dictionary with frame numbers and coordinates + Doppler speed
def extract_coordinates_with_doppler(frames_data, z_threshold=None):
    coordinates_dict = {}

    for frame_num, frame_content in frames_data.items():
        # Extract TLV data
        tlvs = frame_content.get("TLVs", [])

        # Find Type 1 data (Detected Points)
        points = []
        for tlv in tlvs:
            if "Type 1 Data" in tlv:  # Look for Type 1 Data
                points = tlv["Type 1 Data"]
                break  # Assume only one Type 1 entry per frame

        # Create a list of dictionaries with required fields and filters
        coordinates = []
        for point in points:
            # Apply threshold filters
            if z_threshold is not None and not (z_threshold[0] <= point["Z [m]"] <= z_threshold[1]):
                continue  # Skip if Z is outside the range

            # Add the point to the list if it passes all filters
            coordinates.append([
                point["X [m]"],
                point["Y [m]"],
                point["Z [m]"],
                point["Doppler [m/s]"]
            ])

        # Add the filtered coordinates list to the dictionary
        if coordinates:  # Only add frames with valid points
            coordinates_dict[frame_num] = np.array(coordinates)

    return coordinates_dict

# Submap aggregation function
def aggregate_submap(frames_data, start_frame, num_frames=10):
    aggregated_points = []
    for frame in range(start_frame, start_frame + num_frames):
        if frame in frames_data:
            aggregated_points.extend(frames_data[frame])
    return np.array(aggregated_points)

# -------------------------------
# FUNCTION: Plot Clusters
# -------------------------------

# --- Helper Function: Custom Colormap ---
def create_custom_colormap():
    colors = [(1, 1, 1), (1, 0, 0)]  # White to Red
    cmap = LinearSegmentedColormap.from_list("custom_cmap", colors, N=100)
    return cmap, None
# --- Plot Clusters in Cartesian Coordinates ---
def plot_clusters_cartesian(clusters, ax):
    # Set initial view angle (top-down)
    ax.view_init(elev=90, azim=-90)
    for cid, cluster in clusters.items():
        centroid = cluster['centroid']
        priority = cluster['priority']
        points = cluster['points']
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], label=f"Cluster {cid}")
        ax.scatter(centroid[0], centroid[1], centroid[2], c='black', marker='x')
        doppler_avg = np.mean(points[:, 2])
        ax.text(centroid[0] + 0.2, centroid[1] + 0.2, centroid[2] + 0.2, f"{doppler_avg:.2f} m/s", color='purple')
        ax.text(centroid[0] - 0.2, centroid[1] - 0.2, centroid[2] - 0.2, f"P{priority}", color='red')
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        r = [[min_vals[0], max_vals[0]], [min_vals[1], max_vals[1]], [min_vals[2], max_vals[2]]]
        vertices = [
            [r[0][0], r[1][0], r[2][0]], [r[0][1], r[1][0], r[2][0]],
            [r[0][1], r[1][1], r[2][0]], [r[0][0], r[1][1], r[2][0]],
            [r[0][0], r[1][0], r[2][1]], [r[0][1], r[1][0], r[2][1]],
            [r[0][1], r[1][1], r[2][1]], [r[0][0], r[1][1], r[2][1]]
        ]
        edges = [
            [vertices[i] for i in [0, 1, 2, 3]],
            [vertices[i] for i in [4, 5, 6, 7]],
            [vertices[i] for i in [0, 1, 5, 4]],
            [vertices[i] for i in [2, 3, 7, 6]],
            [vertices[i] for i in [1, 2, 6, 5]],
            [vertices[i] for i in [4, 7, 3, 0]]
        ]
        facecolor = 'green' if priority == 3 else 'yellow' if priority == 2 else 'red'
        ax.add_collection3d(Poly3DCollection(edges, alpha=0.2, facecolor=facecolor))

# --- Plot Clusters in Polar Coordinates ---
def plot_clusters_polar(clusters, ax, range_max, range_bins, angle_bins):
    offset = 270
    polar_grid = np.zeros((range_bins, angle_bins))
    for cluster_id, cluster in clusters.items():
        centroid = cluster['centroid']
        priority = cluster['priority']
        r = np.linalg.norm(centroid[:2])
        theta = (np.degrees(np.arctan2(centroid[1], centroid[0])) + offset) % 360

        # Check if the distance is approximately 6m and the angle is between 45 and 315 degrees.
        if np.isclose(r, 6, atol=0.1) and 45 <= theta <= 315:
            print(f"Warning: Cluster {cluster_id} is at ~6m and {theta:.2f}°!")

        if r < range_max:
            r_bin = int(r / (range_max / range_bins))
            theta_bin = int(theta / (360 / angle_bins))
            polar_grid[r_bin, theta_bin] += 1
            color = 'green' if priority == 3 else 'yellow' if priority == 2 else 'red'
            ax.scatter(np.radians(theta), r, color=color, s=70, label=f'Cluster {cluster_id}')
    r = np.linspace(0, range_max, range_bins)
    theta = np.radians(np.linspace(0, 360, angle_bins, endpoint=False))
    R, Theta = np.meshgrid(r, theta)
    cmap, _ = create_custom_colormap()
    ax.pcolormesh(Theta, R, polar_grid.T, cmap=cmap)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(1)
    ax.set_title("Polar Occupancy Grid with Clusters")

# --- Plot Occupancy Grid in Cartesian Coordinates ---
def plot_occupancy_grid_cartesian(grid, ax, x_limits, y_limits, grid_spacing):
    extent = [x_limits[0], x_limits[1], y_limits[0], y_limits[1]]
    cmap, _ = create_custom_colormap()
    ax.imshow(grid.T, extent=extent, origin='lower', cmap=cmap, alpha=0.5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title("Occupancy Grid (Cartesian)")

# Interactive slider-based visualization
def plot_with_slider(frames_data, num_frames=10):
    fig = plt.figure(figsize=(18, 6))
    gs = GridSpec(1, 3, figure=fig)
    ax_cartesian = fig.add_subplot(gs[0, 0], projection='3d')
    ax_polar = fig.add_subplot(gs[0, 1], polar=True)
    ax_occupancy = fig.add_subplot(gs[0, 2])
    ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
    slider = Slider(ax_slider, 'Frame', 1, len(frames_data) - num_frames + 1, valinit=1, valstep=1)

    range_max = 10
    grid_spacing = 0.5
    x_limits = (-10, 10)
    y_limits = (0, 15)
    range_bins = int(range_max / grid_spacing)
    angle_bins = 360

    def update(val):
        start_frame = int(slider.val)
        submap = aggregate_submap(frames_data, start_frame, num_frames)
        ax_cartesian.clear()
        ax_polar.clear()
        ax_occupancy.clear()

        # First Clustering Stage
        clustersStage1, _ = cluster_points(submap, eps=2.0, min_samples=2)
        points_stage1 = np.concatenate([cluster['points'] for cluster in clustersStage1.values()])

        # Second Clustering Stage
        clustersStage2, _ = cluster_points(points_stage1, eps=1.0, min_samples=6)
        points_stage2 = np.concatenate([cluster['points'] for cluster in clustersStage2.values()])

        # Calculate Occupancy Grid using the final clustered points
        occupancy_grid = calculate_occupancy_grid(points_stage2[:, :2], x_limits, y_limits, grid_spacing)

        # Plot using the second stage clusters
        plot_clusters_cartesian(clustersStage2, ax_cartesian)
        plot_clusters_polar(clustersStage2, ax_polar, range_max, range_bins, angle_bins)
        plot_occupancy_grid_cartesian(occupancy_grid, ax_occupancy, x_limits, y_limits, grid_spacing)


        ax_cartesian.set_xlim(-10, 10)
        ax_cartesian.set_ylim(0, 15)
        ax_cartesian.set_zlim(-0.5, 5)
        ax_cartesian.set_title("3D Cluster View")
        ax_polar.set_ylim(0, range_max)
        ax_occupancy.set_xlim(x_limits)
        ax_occupancy.set_ylim(y_limits)
        plt.draw()

    slider.on_changed(update)
    update(1)
    plt.show()


# Example Usage
script_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = os.path.join("..", "..", "..", "Logs", "LogsPart3", "DynamicMonitoring", "30fps_straight_3x3_2_log_2024-12-16.csv")
file_path = os.path.normpath(os.path.join(script_dir, relative_path))

y_threshold = 0.0
z_threshold = (-0.30, 2.0)
doppler_threshold = 0.0

print(f"Processing file: {file_path}")
frames_data = process_log_file(file_path, snr_threshold=12)

coordinates_data = extract_coordinates_with_doppler(frames_data, z_threshold)

# Plot with slider and clustering
plot_with_slider(coordinates_data, num_frames=10)

# Count total rows in the file (excluding header)
total_rows = sum(1 for _ in open(file_path)) - 1

# Print summary
print(f"\nParsed {len(frames_data)} frames successfully out of {total_rows} total rows.")


