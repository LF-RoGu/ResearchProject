import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.gridspec import GridSpec

from frameAggregator import FrameAggregator
import pointFilter
import dbCluster
import occupancyGrid
from fileSearch import find_project_root
from decodeFile import ImuCSVReader
from decodeFile import RadarCSVReader

# -------------------------------------------------------------
# MODULE: Pipeline.py
# PURPOSE: Main processing pipeline for radar point cloud data.
# This includes filtering, clustering, and occupancy grid generation.
# -------------------------------------------------------------

# -------------------------------------------------------------
# CONSTANTS: Simulation parameters
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 1
FILTER_SNR_MIN = 12
FILTER_Z_MIN = -2
FILTER_Z_MAX = 2
FILTER_PHI_MIN = -85
FILTER_PHI_MAX = 85
FILTER_DOPPLER_MIN = 0.0
FILTER_DOPPLER_MAX = 8.0

# -------------------------------------------------------------
# OBJECTS: Processing stages
# -------------------------------------------------------------
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=4)
grid_processor = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)

# -------------------------------------------------------------
# FUNCTION: create_named_subplots
# PURPOSE: Create multiple named subplots with specified projections.
# -------------------------------------------------------------
def create_named_subplots(fig, layout, names, projections):
    """Create dynamic named subplots."""
    gs = GridSpec(*layout, figure=fig)
    axes = {}
    for idx, (name, proj) in enumerate(zip(names, projections)):
        row = idx // layout[1]
        col = idx % layout[1]
        if proj == '3d':
            axes[name] = fig.add_subplot(gs[row, col], projection='3d')
        else:
            axes[name] = fig.add_subplot(gs[row, col])
    return axes

# -------------------------------------------------------------
# FUNCTION: update_sim
# PURPOSE: Simulation update logic for processing frames.
# -------------------------------------------------------------
def update_sim(new_num_frame):
    """Update simulation with new frame number."""
    global curr_num_frame

    if new_num_frame < curr_num_frame:
        frame_aggregator.clearBuffer()
        curr_num_frame = -1

    for num_frame in range(curr_num_frame + 1, new_num_frame + 1, 1):
        frame = radar_frames[num_frame]
        frame_aggregator.updateBuffer(frame)

        point_cloud = frame_aggregator.getPoints()

        # Apply point cloud filtering
        filtered_point_cloud = pointFilter.filterSNRmin(point_cloud, FILTER_SNR_MIN)
        filtered_point_cloud = pointFilter.filterCartesianZ(filtered_point_cloud, FILTER_Z_MIN, FILTER_Z_MAX)
        filtered_point_cloud = pointFilter.filterSphericalPhi(filtered_point_cloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filteredDoppler_point_cloud = pointFilter.filterDoppler(filtered_point_cloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)
    
    # -------------------------------------------------------------
    # VARIABLE: imu_info
    # PURPOSE: Example (x,y,z) info generated randomly for testing.
    # -------------------------------------------------------------
    imu_info = np.random.uniform(low=-10.0, high=10.0, size=3)

    update_graphs(raw_points=point_cloud,
                  filtered_points=filteredDoppler_point_cloud,
                  imu_info=imu_info)
    curr_num_frame = new_num_frame

# -------------------------------------------------------------
# FUNCTION: update_graphs
# PURPOSE: Update the graphs with processed data.
# -------------------------------------------------------------
def update_graphs(raw_points, filtered_points, imu_info):
    """Update all subplots with new data."""

    l_raw_points = pointFilter.extract_points(raw_points)
    l_filtered_points = pointFilter.extract_points(filtered_points)

    # -------------------------------------------------------------
    # FUNCTION: add_corner_text
    # PURPOSE: Add (x,y,z) information to the bottom right corner 
    #          of a plot. This annotation shows IMU or any custom 
    #          information for each frame in a consistent location.
    # PARAMETERS:
    #   ax       - Matplotlib axes object to annotate.
    #   imu_info - Tuple or list of three floats representing (x,y,z).
    # NOTES:
    #   The text box uses axes coordinates so it stays fixed during 
    #   zoom or resize. A semi-transparent background improves 
    #   readability over point clouds.
    # -------------------------------------------------------------
    def add_corner_text(ax, imu_info):
        # Format the (x,y,z) text string with two decimal places.
        text = "(x,y,z) = ({:.2f}, {:.2f}, {:.2f})".format(
            imu_info[0],
            imu_info[1],
            imu_info[2]
        )

        # Add the formatted text to the bottom-right corner.
        # For 3D Axes, use x, y, z, s
        ax.text(
            0.95,          # x position in axes coords
            0.02,          # y position in axes coords
            0.0,           # z position (pick a default plane)
            text,          # string
            transform=ax.transAxes,
            fontsize=10,
            color='black',
            ha='right',
            va='bottom',
            bbox={
                'boxstyle': 'round,pad=0.3',
                'facecolor': 'white',
                'alpha': 0.7
            }
        )

    # -------------------------------------------------------------
    # FUNCTION: plot_3d_points
    # PURPOSE: Plot a 3D point cloud with Doppler speed annotations.
    # -------------------------------------------------------------
    def plot_3d_points(ax, title, points, color='b', imu_info=None):
        """Plot 3D points with labels and axes."""
        ax.clear()
        ax.set_title(title)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_xlim(-10, 10)
        ax.set_ylim(0, 15)
        ax.set_zlim(-2, 10)
        ax.view_init(elev=90, azim=-90)

        points = np.asarray(points)
        if points.ndim == 1 or points.shape[0] == 0:
            return  # No points to plot
        if points.ndim == 1 and points.size == 3:
            points = points.reshape(1, 3)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color)

        # Annotate each point with its Doppler speed if available
        for point in points:
            x, y, z = point[0], point[1], point[2]
            if len(point) >= 4:
                doppler = point[3]
                ax.text(
                    x, y, z + 0.05,  # slight Z offset
                    f"{doppler:.2f} m/s",
                    fontsize=7,
                    color='red'
                )

        if imu_info is not None:
            add_corner_text(ax, imu_info)

    # -------------------------------------------------------------
    # PLOT: Raw-PointCloud with Doppler labels
    # -------------------------------------------------------------
    plot_3d_points(
        axes["Raw-PointCloud"],
        'Raw-PointCloud',
        np.array(l_raw_points),   # Keep Doppler info
        imu_info=imu_info
    )

    # -------------------------------------------------------------
    # PLOT: Filter-PointCloud with Doppler labels
    # -------------------------------------------------------------
    plot_3d_points(
        axes["Filter-PointCloud"],
        'Filter-PointCloud',
        np.array(l_filtered_points),  # Keep Doppler info
        imu_info=imu_info
    )

# -------------------------------------------------------------
# ENTRY POINT: Load data, set up figure and slider.
# -------------------------------------------------------------
radarLoader = RadarCSVReader(file_name="radar_straightWall_1.csv", folder_name="04_Logs-10072025")
imuLoader = ImuCSVReader(file_name="imu_straightWall_1.csv", folder_name="04_Logs-10072025")

imu_frames = imuLoader.load_all()
radar_frames = radarLoader.load_all()

# -------------------------------------------------------------
# OBJECT: FrameAggregator instance
# -------------------------------------------------------------
frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

# -------------------------------------------------------------
# PLOT SETUP: Create subplots and slider
# -------------------------------------------------------------
fig = plt.figure(figsize=(12, 8))
axes = create_named_subplots(
    fig,
    (2, 2),  # Layout with room for 3 subplots
    names=["Raw-PointCloud", "Filter-PointCloud"],
    projections=["3d", "3d"]
)

curr_num_frame = -1

ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(radar_frames) - 1, valinit=0, valstep=1)
slider.on_changed(update_sim)

# -------------------------------------------------------------
# INITIALIZATION: Start simulation at frame 0
# -------------------------------------------------------------
update_sim(0)
plt.show()
