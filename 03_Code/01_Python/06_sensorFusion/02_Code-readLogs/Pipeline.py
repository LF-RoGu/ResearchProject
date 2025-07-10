import os
import numpy as np
import math
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
#
# PURPOSE:
#   - Load IMU + radar logs
#   - Apply filtering, clustering, occupancy grid
#   - Visualize raw vs filtered clouds
#   - Animate over frames with a slider
#
# -------------------------------------------------------------

# -------------------------------------------------------------
# CONSTANTS: Simulation parameters
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 1
FILTER_SNR_MIN                = 12
FILTER_Z_MIN                  = -2
FILTER_Z_MAX                  = 2
FILTER_PHI_MIN                = -85
FILTER_PHI_MAX                = 85
FILTER_DOPPLER_MIN            = 0.0
FILTER_DOPPLER_MAX            = 8.0

# -------------------------------------------------------------
# OBJECTS: Processing stages
# -------------------------------------------------------------
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=4)
grid_processor           = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)

# -------------------------------------------------------------
# FUNCTION: create_named_subplots
# PURPOSE: Create multiple named subplots with specified projections.
# -------------------------------------------------------------
def create_named_subplots(fig, layout, names, projections):
    gs   = GridSpec(*layout, figure=fig)
    axes = {}
    for idx, (name, proj) in enumerate(zip(names, projections)):
        row, col = divmod(idx, layout[1])
        if proj == '3d':
            axes[name] = fig.add_subplot(gs[row, col], projection='3d')
        else:
            axes[name] = fig.add_subplot(gs[row, col])
    return axes


# -------------------------------------------------------------
# FUNCTION: average_imu_records
# PURPOSE: Average numeric fields of a list of ImuRecord objects.
# -------------------------------------------------------------
def average_imu_records(imu_records):
    if not imu_records:
        return None
    numeric_fields = [f for f, v in vars(imu_records[0]).items()
                      if isinstance(v, (int, float))]
    return {f: np.mean([getattr(r, f) for r in imu_records])
            for f in numeric_fields}


# -------------------------------------------------------------
# FUNCTION: update_sim
# PURPOSE: Simulation update logic for processing frames.
# -------------------------------------------------------------
def update_sim(new_frame):
    global curr_num_frame

    if new_frame < curr_num_frame:
        frame_aggregator.clearBuffer()
        curr_num_frame = -1

    for fid in range(curr_num_frame + 1, new_frame + 1):
        imu_list = imu_frames[fid]
        imu_avg  = average_imu_records(imu_list)

        frame_aggregator.updateBuffer(radar_frames[fid])
        raw_pc = frame_aggregator.getPoints()

        # filtering pipeline
        pc = pointFilter.filterSNRmin(raw_pc, FILTER_SNR_MIN)
        pc = pointFilter.filterCartesianZ(pc, FILTER_Z_MIN, FILTER_Z_MAX)
        pc = pointFilter.filterSphericalPhi(pc, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filtered_pc = pointFilter.filterDoppler(pc, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

    # Update the plots with this frame’s data
    update_graphs(raw_pc, filtered_pc, imu_avg)
    curr_num_frame = new_frame


# -------------------------------------------------------------
# FUNCTION: update_graphs
# PURPOSE: Update the graphs with processed data and IMU.
# -------------------------------------------------------------
def update_graphs(raw_points, filtered_points, imu_info):
    l_raw  = pointFilter.extract_points(raw_points)
    l_filt = pointFilter.extract_points(filtered_points)

    def add_corner_text(ax, imu):
        text = (
            "accel (x,y,z):  {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}\n"
            "free  (x,y,z):  {free_accel_x:.2f}, {free_accel_y:.2f}, {free_accel_z:.2f}\n"
            "gyro  (x,y,z):  {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}\n"
            "mag   (x,y,z):  {mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f}\n"
            "temp: {temperature:.2f} °C"
        ).format(**imu)
        ax.text(0.95, 0.02, 0.0, text,
                transform=ax.transAxes,
                fontsize=10,
                ha='right', va='bottom',
                bbox={'boxstyle': 'round,pad=0.3',
                      'facecolor': 'white',
                      'alpha': 0.7})

    def plot_3d_points(ax, title, pts, imu_info):
        ax.clear()
        ax.set_title(title)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_xlim(-10, 10)
        ax.set_ylim(0, 15)
        ax.set_zlim(-2, 10)
        ax.view_init(elev=90, azim=-90)

        arr = np.asarray(pts)
        if arr.size == 0:
            return
        if arr.ndim == 1 and arr.size == 3:
            arr = arr.reshape(1, 3)

        ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2], c='b')
        for pt in arr:
            if len(pt) >= 4:
                ax.text(pt[0], pt[1], pt[2] + 0.05,
                        f"{pt[3]:.2f} m/s", fontsize=7)
        if imu_info is not None:
            add_corner_text(ax, imu_info)

    # -------------------------------------------------------------
    # PLOT: Raw-PointCloud with Doppler labels
    # -------------------------------------------------------------
    plot_3d_points(
        axes["Raw-PointCloud"],
        'Raw-PointCloud',
        np.array(l_raw),
        imu_info=imu_info
    )

    # -------------------------------------------------------------
    # PLOT: Filter-PointCloud with Doppler labels
    # -------------------------------------------------------------
    plot_3d_points(
        axes["Filter-PointCloud"],
        'Filter-PointCloud',
        np.array(l_filt),
        imu_info=imu_info
    )

    # -------------------------------------------------------------
    # 2D IMU‐Direction arrows (new)
    # -------------------------------------------------------------
    ax_dir = axes['IMU-Direction']
    ax_dir.clear()
    ax_dir.set_title(
        f"IMU 2D Heading: {math.degrees(_compute_yaw(imu_info)):.1f}°"
    )
    ax_dir.set_xlim(-1, 1)
    ax_dir.set_ylim(-1, 1)
    ax_dir.set_aspect('equal', 'box')
    ax_dir.grid(True)

    yaw   = _compute_yaw(imu_info)
    x_dir = math.cos(yaw)
    y_dir = math.sin(yaw)
    # draw body X-axis (red)
    ax_dir.arrow(0, 0, x_dir, y_dir,
                 head_width=0.05, length_includes_head=True, color='r')
    # draw body Y-axis (green)
    x2 = -math.sin(yaw)
    y2 =  math.cos(yaw)
    ax_dir.arrow(0, 0, x2, y2,
                 head_width=0.05, length_includes_head=True, color='g')


# -------------------------------------------------------------
# HELPER: compute yaw from IMU quaternion
# -------------------------------------------------------------
def _compute_yaw(imu):
    if (imu['quat_w'] == 0 and imu['quat_x'] == 0 
        and imu['quat_y'] == 0 and imu['quat_z'] == 0):
        # fallback only if quaternion missing
        return math.atan2(imu['mag_y'], imu['mag_x'])
    else:
        qw = imu['quat_w']
        qx = imu['quat_x']
        qy = imu['quat_y']
        qz = imu['quat_z']
        return math.atan2(
            2*(qw*qz + qx*qy),
            1 - 2*(qy*qy + qz*qz)
        )


# -------------------------------------------------------------
# ENTRY POINT: Load data, set up figure and slider.
# -------------------------------------------------------------
radarLoader = RadarCSVReader(
    file_name="radar_driveAround_1.csv",
    folder_name="04_Logs-10072025_v2"
)
imuLoader = ImuCSVReader(
    file_name="imu_driveAround_1.csv",
    folder_name="04_Logs-10072025_v2"
)

imu_frames   = imuLoader.load_all()
radar_frames = radarLoader.load_all()

frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

fig = plt.figure(figsize=(12, 9))
axes = create_named_subplots(
    fig,
    (2, 2),
    names=["Raw-PointCloud", "Filter-PointCloud"],
    projections=["3d", "3d"]
)

# additional 2D axes for IMU direction arrows
axes['IMU-Direction'] = fig.add_axes([0.1, 0.15, 0.8, 0.2])

curr_num_frame = -1

ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider    = Slider(
    ax_slider,
    'Frame',
    0,
    len(radar_frames) - 1,
    valinit=0,
    valstep=1
)
slider.on_changed(update_sim)

# -------------------------------------------------------------
# INITIALIZATION: Start simulation at frame 0
# -------------------------------------------------------------
update_sim(0)
plt.show()
