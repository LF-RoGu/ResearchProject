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
# CONFIGURATION CONSTANTS
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 1
FILTER_SNR_MIN = 12
FILTER_Z_MIN = -2
FILTER_Z_MAX = 2
FILTER_PHI_MIN = -85
FILTER_PHI_MAX = 85
FILTER_DOPPLER_MIN = 0.0
FILTER_DOPPLER_MAX = 8.0

OCCUPANCY_P_THRESHOLD = 0.5  # Threshold for cell occupancy probability

cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=4)
grid_processor = occupancyGrid.OccupancyGridProcessor(
    grid_spacing=0.5,   # grid spacing for each grid
    p_hit=0.7,          # detection trust
    p_miss=0.4          # decay trust
)


# -------------------------------------------------------------
# GLOBAL handles for occupancy grid plot
# -------------------------------------------------------------
im_occ = None   # QuadMesh for imshow
cbar_occ = None # Colorbar handle

# -------------------------------------------------------------
# FUNCTION: Create multiple named subplots
# -------------------------------------------------------------
def create_named_subplots(fig, layout, names, projections):
    """
    Generate subplots with dynamic layout and projections.
    """
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
# FUNCTION: Simulation update logic
# -------------------------------------------------------------
def update_sim(new_num_frame):
    """
    Apply frame update logic, including filtering, clustering,
    and occupancy grid update.
    """
    global curr_num_frame

    if new_num_frame < curr_num_frame:
        frame_aggregator.clearBuffer()
        curr_num_frame = -1

    for num_frame in range(curr_num_frame + 1, new_num_frame + 1, 1):
        frame = radar_frames[num_frame]
        frame_aggregator.updateBuffer(frame)

        point_cloud = frame_aggregator.getPoints()

        filtered_point_cloud = pointFilter.filterSNRmin(point_cloud, FILTER_SNR_MIN)
        filtered_point_cloud = pointFilter.filterCartesianZ(filtered_point_cloud, FILTER_Z_MIN, FILTER_Z_MAX)
        filtered_point_cloud = pointFilter.filterSphericalPhi(filtered_point_cloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filteredDoppler_point_cloud = pointFilter.filterDoppler(filtered_point_cloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

        cluster_input = pointFilter.extract_points(filteredDoppler_point_cloud)
        clusters, _ = cluster_processor_stage2.cluster_points(cluster_input)

        grid_processor.x_limits = (-10, 10)
        grid_processor.y_limits = (0, 15)
        grid_processor.update_log_odds_grid(cluster_input)

    update_graphs(raw_points=point_cloud,
                  filtered_points=filteredDoppler_point_cloud,
                  clusters=clusters)
    curr_num_frame = new_num_frame

# -------------------------------------------------------------
# FUNCTION: Update all subplots in the figure
# PURPOSE: Refresh Raw, Filter, DB Clusters, and Occupancy Grid.
# -------------------------------------------------------------
def update_graphs(raw_points, filtered_points, clusters):
    global im_occ, cbar_occ  # Handles for occupancy grid image and colorbar

    # -------------------------------------------------------------
    # Extract point arrays for plotting
    # -------------------------------------------------------------
    l_raw_points = pointFilter.extract_points(raw_points)
    l_filtered_points = pointFilter.extract_points(filtered_points)

    # -------------------------------------------------------------
    # FUNCTION: Helper to plot a 3D scatter plot
    # -------------------------------------------------------------
    def plot_3d_points(ax, title, points, color='b'):
        """
        Plot 3D points with consistent limits and view.
        """
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
            return  # Nothing to plot
        if points.ndim == 1 and points.size == 3:
            points = points.reshape(1, 3)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color)

    # -------------------------------------------------------------
    # Plot: Raw-PointCloud
    # -------------------------------------------------------------
    plot_3d_points(axes["Raw-PointCloud"], 'Raw-PointCloud', l_raw_points)

    # -------------------------------------------------------------
    # Plot: Filter-PointCloud
    # -------------------------------------------------------------
    plot_3d_points(axes["Filter-PointCloud"], 'Filter-PointCloud', l_filtered_points)

    # -------------------------------------------------------------
    # Plot: DB-Clusters with unique IDs and colors
    # -------------------------------------------------------------
    ax = axes["DB-Clusters"]
    ax.clear()
    ax.set_title("DB-Clusters")
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_xlim(-10, 10)
    ax.set_ylim(0, 15)
    ax.set_zlim(-2, 10)
    ax.view_init(elev=90, azim=-90)

    colors = plt.cm.get_cmap('tab20', len(clusters))
    for idx, (cluster_id, data) in enumerate(clusters.items()):
        cluster_points = data['points']
        ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2],
                   color=colors(idx), label=f'ID {cluster_id}')
        centroid = data['centroid']
        ax.text(centroid[0], centroid[1], centroid[2], f'ID {cluster_id}', color='black')
    ax.legend()

    # -------------------------------------------------------------
    # Plot: Occupancy Grid - Log-Odds with single colorbar
    # -------------------------------------------------------------
    ax_occ = axes["Occupancy-Grid"]
    ax_occ.set_title("Occupancy Grid - Log-Odds")
    ax_occ.set_xlabel("X Position (Grid)")
    ax_occ.set_ylabel("Y Position (Grid)")

    prob_grid = grid_processor.get_occupancy_probability_grid()

    # Only create imshow and colorbar once
    if im_occ is None:
        im_occ = ax_occ.imshow(
            prob_grid.T,
            cmap=grid_processor.cmap,
            norm=grid_processor.norm,
            origin='lower',
            extent=[
                grid_processor.x_limits[0],
                grid_processor.x_limits[1],
                grid_processor.y_limits[0],
                grid_processor.y_limits[1]
            ],
            aspect='auto'
        )
        cbar_occ = fig.colorbar(im_occ, ax=ax_occ, fraction=0.046, pad=0.04)
        ax_occ.set_aspect('equal')
    else:
        im_occ.set_data(prob_grid.T)
        im_occ.set_clim(0.0, 1.0)

# -------------------------------------------------------------
# ENTRY POINT
# -------------------------------------------------------------
radarLoader = RadarCSVReader(file_name="radar_testReflectiveness12.csv", folder_name="03_Logs-07042025")
imuLoader = ImuCSVReader(file_name="imu_testReflectiveness12.csv", folder_name="03_Logs-07042025")
imu_frames = imuLoader.load_all()
radar_frames = radarLoader.load_all()

frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

fig = plt.figure(figsize=(12, 8))
axes = create_named_subplots(
    fig,
    (2, 2),
    names=["Raw-PointCloud", "Filter-PointCloud", "DB-Clusters", "Occupancy-Grid"],
    projections=["3d", "3d", "3d", None]
)

curr_num_frame = -1
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(radar_frames) - 1, valinit=0, valstep=1)
slider.on_changed(update_sim)

update_sim(0)
plt.show()
