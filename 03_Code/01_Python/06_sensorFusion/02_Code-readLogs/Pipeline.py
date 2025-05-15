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
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 1
FILTER_SNR_MIN = 12
FILTER_Z_MIN = 0
FILTER_Z_MAX = 2
FILTER_PHI_MIN = -85
FILTER_PHI_MAX = 85
FILTER_DOPPLER_MIN = 0.1
FILTER_DOPPLER_MAX = 2.0
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
        frame = radar_frames[num_frame]
        frame_aggregator.updateBuffer(frame)
        point_cloud = frame_aggregator.getPoints()

        filtered_point_cloud = pointFilter.filterSNRmin(point_cloud, FILTER_SNR_MIN)
        filtered_point_cloud = pointFilter.filterCartesianZ(filtered_point_cloud, FILTER_Z_MIN, FILTER_Z_MAX)
        filtered_point_cloud = pointFilter.filterSphericalPhi(filtered_point_cloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filteredDoppler_point_cloud = pointFilter.filterDoppler(filtered_point_cloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

        self_speed_raw = selfSpeedEstimator.estimate_self_speed(point_cloud)
        self_speed_filtered = self_speed_kf.update(self_speed_raw)

        self_speed_raw_history.append(self_speed_raw)
        self_speed_filtered_history.append(self_speed_filtered)

    update_graphs(raw_points=point_cloud, filtered_points=filteredDoppler_point_cloud, raw_self_speed_history=self_speed_raw_history, filtered_self_speed_history=self_speed_filtered_history)
    curr_num_frame = new_num_frame

# -------------------------------
# Graph update logic
# -------------------------------
def update_graphs(raw_points, filtered_points, raw_self_speed_history, filtered_self_speed_history):
    global axes, radar_frames
    l_raw_points = pointFilter.extract_points(raw_points)
    l_filtered_points = pointFilter.extract_points(filtered_points)

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

    plot_3d_points(axes["raw point-cloud"], 'Point Cloud', np.array([[p[0], p[1], p[2]] for p in l_raw_points]).reshape(-1, 3))
    plot_3d_points(axes["filtered point-cloud"], 'Point Cloud', np.array([[p[0], p[1], p[2]] for p in l_filtered_points]).reshape(-1, 3))


    axes["ve"].clear()
    axes["ve"].set_title('Vehicle Ve')
    axes["ve"].set_xlim(0, len(radar_frames))
    axes["ve"].set_ylim(-3, 0)
    axes["ve"].plot(np.arange(len(raw_self_speed_history)), raw_self_speed_history, linestyle='--')
    axes["ve"].plot(np.arange(len(filtered_self_speed_history)), filtered_self_speed_history)

# -------------------------------
# Program entry point
# -------------------------------
radarLoader = RadarCSVReader(file_name="radar_data_driveStraight_inclinedSensor_v2.csv", folder_name="03_Logs-15052025")
imuLoader = ImuCSVReader(file_name="imu_data_driveStraight_inclinedSensor_v2.csv", folder_name="03_Logs-15052025")

imu_frames = imuLoader.load_all()
radar_frames = radarLoader.load_all()
# ---------------------------------------------------------
frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
self_speed_kf = KalmanFilter(process_variance=KALMAN_FILTER_PROCESS_VARIANCE, measurement_variance=KALMAN_FILTER_MEASUREMENT_VARIANCE)

fig = plt.figure(figsize=(10, 10))
axes = create_named_subplots(
    fig,
    (2, 3),
    names=["raw point-cloud", "filtered point-cloud", "ve"],
    projections=["3d", "3d", None]
)

curr_num_frame = -1
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(radar_frames) - 1, valinit=0, valstep=1)
slider.on_changed(update_sim)

update_sim(0)
plt.show()
