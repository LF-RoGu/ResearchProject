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
import dbCluster

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
# DEBUG SWITCH: Toggle detailed cluster debug logs
# -------------------------------------------------------------
DEBUG_CLUSTER   = False  # Set to False to disable debug logs
DEBUG_VXVY      = True

# -------------------------------------------------------------
# CONSTANTS: Simulation parameters
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES    = 5
FILTER_SNR_MIN                      = 12
FILTER_Z_MIN                        = -2
FILTER_Z_MAX                        = 2
FILTER_Y_MIN                        = 0.6
FILTER_Y_MAX                        = 15
FILTER_PHI_MIN                      = -85
FILTER_PHI_MAX                      = 85
FILTER_DOPPLER_MIN                  = 0.0
FILTER_DOPPLER_MAX                  = 8.0

# -------------------------------------------------------------
# OBJECTS: Processing stages
# -------------------------------------------------------------
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=4)
grid_processor           = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)


# -------------------------------------------------------------
# DEBUG VAR: Variables for debugging
# -------------------------------------------------------------
# Global variable to hold clusters per frame
current_frame_clusters = []

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
        raw_pointCloud = frame_aggregator.getPoints()

        # ------------------------
        # FILTERING PIPELINE
        # ------------------------
        pointCloud = pointFilter.filterSNRmin(raw_pointCloud, FILTER_SNR_MIN)
        pointCloud = pointFilter.filterCartesianZ(pointCloud, FILTER_Z_MIN, FILTER_Z_MAX)
        pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)
        pointCloud = pointFilter.filterSphericalPhi(pointCloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filtered_pointCloud = pointFilter.filterDoppler(pointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

        # ------------------------
        # STAGE 1 CLUSTERING
        # ------------------------
        points_for_clustering = pointFilter.extract_points(filtered_pointCloud)

        if DEBUG_CLUSTER:
            print(f"\n[DEBUG] Stage 1: Points for clustering shape: {points_for_clustering.shape}")
            if points_for_clustering.size > 0:
                print(f"[DEBUG] Stage 1: Example point: {points_for_clustering[0]}")  # [x,y,z,doppler]

        cluster_pointCloud_stage1, _ = cluster_processor_stage1.cluster_points(points_for_clustering)

        if DEBUG_CLUSTER:
            print(f"[DEBUG] Stage 1: Number of clusters found: {len(cluster_pointCloud_stage1)}")
            for cid, cdata in cluster_pointCloud_stage1.items():
                print(f"\n[DEBUG] Stage 1 - Cluster {cid}: {cdata['points'].shape} points")
                for pt in cdata['points']:
                    print(f"   {pt}")  # [x,y,z,doppler]

        # ------------------------
        # STAGE 2 CLUSTERING (optional)
        # ------------------------
        if len(cluster_pointCloud_stage1) > 0:
            points_stage1_flat = np.vstack([cdata['points'] for cdata in cluster_pointCloud_stage1.values()])

            if DEBUG_CLUSTER:
                print(f"\n[DEBUG] Stage 2: Points for clustering shape: {points_stage1_flat.shape}")
                if points_stage1_flat.size > 0:
                    print(f"[DEBUG] Stage 2: Example point: {points_stage1_flat[0]}")

            cluster_pointCloud_stage2, _ = cluster_processor_stage2.cluster_points(points_stage1_flat)

            if DEBUG_CLUSTER:
                print(f"[DEBUG] Stage 2: Number of clusters found: {len(cluster_pointCloud_stage2)}")
                for cid, cdata in cluster_pointCloud_stage2.items():
                    print(f"\n[DEBUG] Stage 2 - Cluster {cid}: {cdata['points'].shape} points")
                    for pt in cdata['points']:
                        print(f"   {pt}")

            # ------------------------
            # NEW: Compute mean (v_x, v_y) for each cluster
            # ------------------------
            for cid, cdata in cluster_pointCloud_stage2.items():
                points = cdata['points']   # shape (N, 4): [x,y,z,doppler]

                x = points[:, 0]
                y = points[:, 1]
                phis = np.arctan2(x, y)   # Y is North → atan2(x, y)

                dopplers = points[:, 3]

                A = np.stack([np.cos(phis), np.sin(phis)], axis=1)
                R = dopplers.reshape(-1, 1)

                V, _, _, _ = np.linalg.lstsq(A, R, rcond=None)
                v_x, v_y = V.flatten() # gives [v_x, v_y] as float type values

                # ------------------------
                # Validation of prediction of measurements for validation purposes
                # ------------------------
                predicted_R = v_x * np.cos(phis) + v_y * np.sin(phis)
                mean_measured = np.mean(dopplers)
                mean_predicted = np.mean(predicted_R)
                rms_error = np.sqrt(np.mean((dopplers - predicted_R)**2))

                if DEBUG_VXVY:
                    print(f"[DEBUG_VXVY] Cluster {cid} mean Cartesian speed vector: "
                          f"Vx={v_x:.2f} m/s, Vy={v_y:.2f} m/s")
                    print("----------------------------------------------------------")
                    print(f"[DEBUG_VXVY] Cluster {cid}: mean Doppler measured = {mean_measured:.2f} m/s")
                    print(f"[DEBUG_VXVY] Cluster {cid}: mean Doppler predicted = {mean_predicted:.2f} m/s")
                    # Low RMS error → good single rigid-body motion
                    # High RMS error → suspicious cluster
                    print(f"[DEBUG_VXVY] Cluster {cid}: RMS Doppler error = {rms_error:.4f} m/s") 

                # Optional: add to cluster data if you want to store it
                cdata['mean_vx'] = v_x
                cdata['mean_vy'] = v_y
                cdata['cluster_id'] = cid

            final_clusters = cluster_pointCloud_stage2

        else:
            final_clusters = {}

    # ------------------------
    # STORE FINAL CLUSTERS
    # ------------------------
    current_frame_clusters.clear()
    for cid, cdata in final_clusters.items():
        current_frame_clusters.append({
            'cluster_id': cid,
            'points': cdata['points'],
            'centroid': cdata['centroid'],
            'priority': cdata['priority'],
            'doppler_avg': cdata['doppler_avg'],
            'mean_vx': cdata.get('mean_vx', 0.0),
            'mean_vy': cdata.get('mean_vy', 0.0)
        })

    # ------------------------
    # UPDATE PLOTS
    # ------------------------
    update_graphs(
        raw_var=raw_pointCloud,
        filtered_var=filtered_pointCloud,
        cluster_var=final_clusters,
        imu_var=imu_avg
    )

    curr_num_frame = new_frame

# -------------------------------------------------------------
# FUNCTION: update_graphs
# PURPOSE: Update the graphs with processed data and IMU.
# -------------------------------------------------------------
def update_graphs(raw_var, filtered_var, cluster_var, imu_var):
    l_rawData  = pointFilter.extract_points(raw_var)
    l_filteredData = pointFilter.extract_points(filtered_var)
    l_clusterData = pointFilter.extract_points(cluster_var)

    MMWAVE_FPS = 30
    N_frames = FRAME_AGGREGATOR_NUM_PAST_FRAMES
    DT = N_frames / MMWAVE_FPS


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

    def plot_3d_points(ax, title, pts, imu_var):
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
        if imu_var is not None:
            add_corner_text(ax, imu_var)

    # -------------------------------------------------------------
    # PLOT: Raw-PointCloud with Doppler labels
    # -------------------------------------------------------------
    plot_3d_points(
        axes["Raw-PointCloud"],
        'Raw-PointCloud',
        np.array(l_rawData),
        imu_var=imu_var
    )

    # -------------------------------------------------------------
    # PLOT: Filter-PointCloud with Doppler labels
    # -------------------------------------------------------------
    plot_3d_points(
        axes["Filter-PointCloud"],
        'Filter-PointCloud',
        np.array(l_filteredData),
        imu_var=imu_var
    )
    # -------------------------------------------------------------
    # PLOT: DB-Clustered-PointCloud with Doppler labels
    # -------------------------------------------------------------
    priority_colors = {1: 'red', 2: 'orange', 3: 'green'}

    ax_cluster = axes['DB-Clustered-PointCloud']
    ax_cluster.clear()
    ax_cluster.set_title('DB-Clustered-PointCloud')
    ax_cluster.set_xlabel('X [m]')
    ax_cluster.set_ylabel('Y [m]')
    ax_cluster.set_zlabel('Z [m]')
    ax_cluster.set_xlim(-10, 10)
    ax_cluster.set_ylim(0, 15)
    ax_cluster.set_zlim(-2, 10)
    ax_cluster.view_init(elev=90, azim=-90)

    if cluster_var and isinstance(cluster_var, dict):
        for _, cluster_data in cluster_var.items():
            points = cluster_data['points']
            centroid = cluster_data['centroid']
            priority = cluster_data['priority']
            doppler_avg = cluster_data['doppler_avg']

            vx = cluster_data.get('mean_vx', 0.0)
            vy = cluster_data.get('mean_vy', 0.0)

            ax_cluster.quiver(
                centroid[0], centroid[1], centroid[2],  # start
                vx, vy, 0,                               # vector components
                length=2.0, # Lenght of the arrow
                normalize=True,
                color='red',
                arrow_length_ratio=0.2
            )

            color = priority_colors.get(priority, 'gray')
            ax_cluster.scatter(points[:, 0], points[:, 1], points[:, 2],
                               c=color, s=8, alpha=0.7)

            ax_cluster.text(
                centroid[0] + 0.2, centroid[1] + 0.2, centroid[2] + 0.2,
                f"ID:{cluster_data['cluster_id']} P:{priority} {doppler_avg:.2f} m/s",
                fontsize=7,
                color='purple'
            )
            ax_cluster.text(
                centroid[0] + vx * DT,
                centroid[1] + vy * DT,
                centroid[2] + 0.2,
                f"Vx:{vx:.2f} Vy:{vy:.2f}",
                fontsize=7,
                color='red'
            )
    else:
        ax_cluster.text(0, 0, 0, 'No Clusters Detected', fontsize=12, color='red')
    # -------------------------------------------------------------
    # PLOT: 2D IMU‐Direction arrows
    # -------------------------------------------------------------
    ax_dir = axes['IMU-Direction']
    ax_dir.clear()
    ax_dir.set_title(
        f"IMU 2D Heading: {math.degrees(_compute_yaw(imu_var)):.1f}°"
    )
    ax_dir.set_xlim(-1, 1)
    ax_dir.set_ylim(-1, 1)
    ax_dir.set_aspect('equal', 'box')
    ax_dir.grid(True)

    yaw   = _compute_yaw(imu_var)
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
    

    # -----------------------------
    # PLOT: Estimated Clusters
    # -----------------------------
    ax_estimation = axes['Estimated-Clusters']
    ax_estimation.clear()
    ax_estimation.set_title('Estimated Clusters')
    ax_estimation.set_xlabel('X [m]')
    ax_estimation.set_ylabel('Y [m]')
    ax_estimation.set_zlabel('Z [m]')
    ax_estimation.set_xlim(-10, 10)
    ax_estimation.set_ylim(0, 15)
    ax_estimation.set_zlim(-2, 10)
    ax_estimation.view_init(elev=90, azim=-90)


    # Keep track so we only add legend labels once
    legend_added = {'original': False, 'estimated': False}

    if cluster_var and isinstance(cluster_var, dict):
        for _, cluster_data in cluster_var.items():
            points = cluster_data['points']
            centroid = cluster_data['centroid']
            vx = cluster_data.get('mean_vx', 0.0)
            vy = cluster_data.get('mean_vy', 0.0)

            # Predict next centroid
            pred_centroid = estimated_next_position(centroid, vx, vy, DT)

            # Shift all points
            pred_points = np.copy(points)
            pred_points[:, 0] += vx * DT
            pred_points[:, 1] += vy * DT

            # Original cluster: blue
            ax_estimation.scatter(
                points[:, 0], points[:, 1], points[:, 2],
                c='blue', s=8, alpha=0.4,
                label='Current Cluster' if not legend_added['original'] else ""
            )
            legend_added['original'] = True

            # Estimated cluster: lime
            ax_estimation.scatter(
                pred_points[:, 0], pred_points[:, 1], pred_points[:, 2],
                c='lime', s=8, alpha=0.7,
                label='Estimated Next Position' if not legend_added['estimated'] else ""
            )
            legend_added['estimated'] = True

            # Arrow for the vector
            ax_estimation.quiver(
                centroid[0], centroid[1], centroid[2], # start at current centroid
                vx, vy, 0, # vector components in X,Y,Z
                length=DT, normalize=False, color='red'
            ) # The estimated motion direction for the cluster.

            # Label the estimated centroid
            ax_estimation.text(
                pred_centroid[0], pred_centroid[1], pred_centroid[2] + 0.1,
                f"ID:{cluster_data['cluster_id']}",
                fontsize=7, color='purple'
            )

    # Add legend
    ax_estimation.legend(loc='upper right', fontsize=7)


                 


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
# HELPER: compute estimation of next cluster
# -------------------------------------------------------------
def estimated_next_position(centroid, vx, vy, dt):
    # Only X,Y because you’re using 2D motion
    estimated = np.array(centroid[:2]) + dt * np.array([vx, vy])
    # Keep Z as same
    return np.array([estimated[0], estimated[1], centroid[2]])


# -------------------------------------------------------------
# ENTRY POINT: Load data, set up figure and slider.
# -------------------------------------------------------------
radarLoader = RadarCSVReader(
    file_name="radar_straightWall_1.csv",
    folder_name="04_Logs-10072025_v2"
)
imuLoader = ImuCSVReader(
    file_name="imu_straightWall_1.csv",
    folder_name="04_Logs-10072025_v2"
)

imu_frames   = imuLoader.load_all()
radar_frames = radarLoader.load_all()

frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

fig = plt.figure(figsize=(12, 9))
axes = create_named_subplots(
    fig,
    (2, 2),
    names=["Raw-PointCloud", "Filter-PointCloud", "DB-Clustered-PointCloud", "Estimated-Clusters"],
    projections=["3d", "3d", "3d", "3d"]
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
