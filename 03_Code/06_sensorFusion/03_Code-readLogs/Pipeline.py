import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.gridspec import GridSpec

from frameAggregator import FrameAggregator
import pointFilter
import selfSpeedEstimator
from kalmanFilter import KalmanFilter
import veSpeedFilter
import dbCluster
import occupancyGrid
from fileSearch import find_project_root
from decodeFile import ImuCSVReader
from decodeFile import RadarCSVReader

# -------------------------------
# Simulation parameters
# -------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 9
FILTER_SNR_MIN = 12
FILTER_Z_MIN = -0.3
FILTER_Z_MAX = 2
FILTER_PHI_MIN = -85
FILTER_PHI_MAX = 85
KALMAN_FILTER_PROCESS_VARIANCE = 0.01
KALMAN_FILTER_MEASUREMENT_VARIANCE = 0.1

cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=4)
grid_processor = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)

# -------------------------------
# Helper: Create dynamic subplots
# -------------------------------
def create_named_subplots(fig, layout, names, projections):
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

# -------------------------------
# Simulation update logic
# -------------------------------
self_speed_raw_history = []
self_speed_filtered_history = []

def update_sim(new_num_frame):
    global curr_num_frame
    global self_speed_raw_history
    global self_speed_filtered_history

    if new_num_frame < curr_num_frame:
        frame_aggregator.clearBuffer()
        self_speed_kf.clear()
        self_speed_raw_history.clear()
        self_speed_filtered_history.clear()
        curr_num_frame = -1

    for num_frame in range(curr_num_frame + 1, new_num_frame + 1, 1):
        frame = frames[num_frame]
        frame_aggregator.updateBuffer(frame)
        point_cloud = frame_aggregator.getPoints()

        point_cloud_filtered = pointFilter.filterSNRmin(point_cloud, FILTER_SNR_MIN)
        point_cloud_filtered = pointFilter.filterCartesianZ(point_cloud_filtered, FILTER_Z_MIN, FILTER_Z_MAX)
        point_cloud_filtered = pointFilter.filterSphericalPhi(point_cloud_filtered, FILTER_PHI_MIN, FILTER_PHI_MAX)

        self_speed_raw = selfSpeedEstimator.estimate_self_speed(point_cloud_filtered)
        self_speed_filtered = self_speed_kf.update(self_speed_raw)

        point_cloud_ve = veSpeedFilter.calculateVe(point_cloud_filtered)
        point_cloud_ve_filtered = pointFilter.filter_by_speed(point_cloud_filtered, self_speed_filtered, 1.0)

        pc_stage1 = pointFilter.extract_points(point_cloud_ve_filtered)
        clusters_stage1, _ = cluster_processor_stage1.cluster_points(pc_stage1)
        pc_stage2 = pointFilter.extract_points(clusters_stage1)
        clusters_stage2, _ = cluster_processor_stage2.cluster_points(pc_stage2)
        point_cloud_clustered = clusters_stage2

        self_speed_raw_history.append(self_speed_raw)
        self_speed_filtered_history.append(self_speed_filtered)

    update_graphs(point_cloud, point_cloud_filtered, point_cloud_ve_filtered, self_speed_raw_history, self_speed_filtered_history, point_cloud_clustered)
    curr_num_frame = new_num_frame

# -------------------------------
# Graph update logic
# -------------------------------
def update_graphs(raw_points, point_cloud_points, filtered_points, self_speed_raw_history, self_speed_filtered_history, cluster_points):
    global axes, frames
    point_cloud_clustered = pointFilter.extract_points(cluster_points)

    def plot_3d_points(ax, title, points, color='b'):
        ax.clear()
        ax.set_title(title)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_xlim(-10, 10)
        ax.set_ylim(0, 15)
        ax.set_zlim(-0.30, 10)
        ax.view_init(elev=90, azim=-90)

        points = np.asarray(points)
        if points.ndim == 1 or points.shape[0] == 0:
            return  # Nothing to plot
        if points.ndim == 1 and points.size == 3:
            points = points.reshape(1, 3)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color)

    plot_3d_points(axes["filtered"], 'Point Cloud Filters - Ve Filters', np.array([[p['x'], p['y'], p['z']] for p in filtered_points]).reshape(-1, 3)
    )

    axes["ve"].clear()
    axes["ve"].set_title('Vehicle Ve')
    axes["ve"].set_xlim(0, len(frames))
    axes["ve"].set_ylim(-3, 0)
    axes["ve"].plot(np.arange(len(self_speed_raw_history)), self_speed_raw_history, linestyle='--')
    axes["ve"].plot(np.arange(len(self_speed_filtered_history)), self_speed_filtered_history)

    ax_cluster = axes["dbCluster"]
    ax_cluster.clear()
    ax_cluster.set_title('Clustered Point Cloud')
    ax_cluster.set_xlabel('X [m]')
    ax_cluster.set_ylabel('Y [m]')
    ax_cluster.set_zlabel('Z [m]')
    ax_cluster.set_xlim(-10, 10)
    ax_cluster.set_ylim(0, 15)
    ax_cluster.set_zlim(-0.30, 10)
    ax_cluster.view_init(elev=90, azim=-90)

    priority_colors = {1: 'red', 2: 'orange', 3: 'green'}
    if cluster_points:
        for cluster_data in cluster_points.values():
            centroid = cluster_data['centroid']
            priority = cluster_data['priority']
            points = cluster_data['points']
            doppler_avg = cluster_data['doppler_avg']
            color = priority_colors.get(priority, 'gray')
            ax_cluster.scatter(points[:, 0], points[:, 1], points[:, 2], c=color, s=8, alpha=0.7)
            ax_cluster.text(centroid[0] + 0.2, centroid[1] + 0.2, centroid[2] + 0.2, f"{doppler_avg:.2f} m/s", color='purple')
    else:
        ax_cluster.text(0, 0, 0, 'No Clusters Detected', fontsize=12, color='red')

    ax_grid = axes["occupancyGrid"]
    ax_grid.clear()
    ax_grid.set_title('Occupancy Grid')
    ax_grid.set_xlabel('X [m]')
    ax_grid.set_ylabel('Y [m]')
    if point_cloud_clustered.size > 0:
        occupancy_grid = grid_processor.calculate_cartesian_grid(point_cloud_clustered[:, :2], x_limits=(-10, 10), y_limits=(0, 15))
        ax_grid.imshow(occupancy_grid.T, cmap=grid_processor.cmap, norm=grid_processor.norm, origin='lower', extent=(-10, 10, 0, 15))
    else:
        ax_grid.set_title('Occupancy Grid (No Data)')

# -------------------------------
# Program entry point
# -------------------------------
radarLoader = RadarCSVReader(file_name="radar_data_30_04_2025.csv")
imuLoader = ImuCSVReader(file_name="imu_data_30_04_2025.csv")

imu_frames = ImuCSVReader.load_all()
radar_frames = RadarCSVReader.load_all()

frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
self_speed_kf = KalmanFilter(process_variance=KALMAN_FILTER_PROCESS_VARIANCE, measurement_variance=KALMAN_FILTER_MEASUREMENT_VARIANCE)

fig = plt.figure(figsize=(10, 10))
axes = create_named_subplots(
    fig,
    (2, 3),
    names=["dbCluster", "filtered", "ve", "occupancyGrid"],
    projections=["3d", "3d", None, None]
)

curr_num_frame = -1
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(radar_frames) - 1, valinit=0, valstep=1)
slider.on_changed(update_sim)

update_sim(0)
plt.show()
