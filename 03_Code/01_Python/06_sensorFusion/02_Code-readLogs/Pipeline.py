"""
Pipeline.py
-------------------------------------------------------------
PURPOSE:
  Main radar point cloud pipeline with:
   - Frame aggregation
   - SNR, Z, Phi, Doppler filtering
   - DB Clustering with unique IDs
   - Log-odds Bayesian occupancy grid
   - Digital grid thresholding and history buffer
   - Visual debug panel for occupancy history info
-------------------------------------------------------------
"""

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

# Log-odds occupancy grid config
OCCUPANCY_P_THRESHOLD = 0.4     # Minimum P(occupied) for binary grid
OCCUPANCY_HISTORY_SIZE = 300     # How many binary grids to store

# -------------------------------------------------------------
# PROCESSOR OBJECTS
# -------------------------------------------------------------
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=4)
grid_processor = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)

# -------------------------------------------------------------
# GLOBAL handles for occupancy grid and history
# -------------------------------------------------------------
im_occ = None          # Imshow handle for occupancy grid
cbar_occ = None        # Colorbar handle
occupancy_history = [] # Time-series storage of binary occupancy grids

# -------------------------------------------------------------
# FUNCTION: Create named subplots with dynamic projections
# -------------------------------------------------------------
def create_named_subplots(fig, layout, names, projections):
    """
    Create a GridSpec layout and return named axes dict.
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
# FUNCTION: Update simulation for given frame number
# -------------------------------------------------------------
def update_sim(new_num_frame):
    """
    Process frames:
     - Aggregate
     - Filter
     - Cluster
     - Update occupancy grid
     - Threshold binary map and store history
    """
    global curr_num_frame

    if new_num_frame < curr_num_frame:
        frame_aggregator.clearBuffer()
        curr_num_frame = -1

    for num_frame in range(curr_num_frame + 1, new_num_frame + 1, 1):
        frame = radar_frames[num_frame]
        frame_aggregator.updateBuffer(frame)

        point_cloud = frame_aggregator.getPoints()

        # Apply filters
        filtered_point_cloud = pointFilter.filterSNRmin(point_cloud, FILTER_SNR_MIN)
        filtered_point_cloud = pointFilter.filterCartesianZ(filtered_point_cloud, FILTER_Z_MIN, FILTER_Z_MAX)
        filtered_point_cloud = pointFilter.filterSphericalPhi(filtered_point_cloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filteredDoppler_point_cloud = pointFilter.filterDoppler(filtered_point_cloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

        # Clustering
        cluster_input = pointFilter.extract_points(filteredDoppler_point_cloud)
        clusters, _ = cluster_processor_stage2.cluster_points(cluster_input)

        # Occupancy grid update
        grid_processor.x_limits = (-10, 10)
        grid_processor.y_limits = (0, 15)
        grid_processor.update_log_odds_grid(cluster_input)

        # Binary occupancy grid & store in history
        P_grid = grid_processor.get_occupancy_probability_grid()
        binary_grid = (P_grid >= OCCUPANCY_P_THRESHOLD).astype(int)
        occupancy_history.append(binary_grid.copy())

        if len(occupancy_history) > OCCUPANCY_HISTORY_SIZE:
            occupancy_history.pop(0)

    update_graphs(raw_points=point_cloud,
                  filtered_points=filteredDoppler_point_cloud,
                  clusters=clusters)
    curr_num_frame = new_num_frame

# -------------------------------------------------------------
# FUNCTION: Update all plots each frame
# -------------------------------------------------------------
def update_graphs(raw_points, filtered_points, clusters):
    """
    Refresh:
     - Raw point cloud
     - Filtered point cloud
     - DB Clusters with unique IDs
     - Occupancy grid with colorbar
     - Text panel for occupancy history
    """
    global im_occ, cbar_occ

    l_raw_points = pointFilter.extract_points(raw_points)
    l_filtered_points = pointFilter.extract_points(filtered_points)

    # -------------------------------------------------------------
    # Helper: 3D scatter plot
    # -------------------------------------------------------------
    def plot_3d_points(ax, title, points, color='b'):
        """
        MISRA: Plot 3D points with fixed view and limits.
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
            return
        if points.ndim == 1 and points.size == 3:
            points = points.reshape(1, 3)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color)

    # Raw-PointCloud
    plot_3d_points(axes["Raw-PointCloud"], 'Raw-PointCloud', l_raw_points)

    # Filter-PointCloud
    plot_3d_points(axes["Filter-PointCloud"], 'Filter-PointCloud', l_filtered_points)

    # DB Clusters
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

    # Occupancy Grid
    ax_occ = axes["Occupancy-Grid"]
    ax_occ.set_title("Occupancy Grid - Log-Odds")
    ax_occ.set_xlabel("X Position (Grid)")
    ax_occ.set_ylabel("Y Position (Grid)")

    prob_grid = grid_processor.get_occupancy_probability_grid()

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
    # Occupancy History Info Text Panel
    # -------------------------------------------------------------
    ax_hist = axes["Occupancy-History"]
    ax_hist.clear()
    ax_hist.set_title("Occupancy History Info")
    ax_hist.axis('off')

    history_size = len(occupancy_history)
    latest_occupied = np.sum(occupancy_history[-1]) if history_size > 0 else 0

    debug_text = f"""
Frames stored: {history_size}
Occupied cells (latest): {latest_occupied}
Occupancy threshold: {OCCUPANCY_P_THRESHOLD}
"""

    ax_hist.text(0.1, 0.6, debug_text, fontsize=12, va='top', ha='left', wrap=True)

# -------------------------------------------------------------
# ENTRY POINT
# -------------------------------------------------------------
radarLoader = RadarCSVReader(file_name="radar_testReflectiveness12.csv", folder_name="03_Logs-07042025")
imuLoader = ImuCSVReader(file_name="imu_testReflectiveness12.csv", folder_name="03_Logs-07042025")
imu_frames = imuLoader.load_all()
radar_frames = radarLoader.load_all()

frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

fig = plt.figure(figsize=(14, 10))
axes = create_named_subplots(
    fig,
    (3, 2),  # 3 rows x 2 columns to fit 5 plots
    names=["Raw-PointCloud", "Filter-PointCloud", "DB-Clusters", "Occupancy-Grid", "Occupancy-History"],
    projections=["3d", "3d", "3d", None, None]
)

curr_num_frame = -1
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(radar_frames) - 1, valinit=0, valstep=1)
slider.on_changed(update_sim)

update_sim(0)
plt.show()
