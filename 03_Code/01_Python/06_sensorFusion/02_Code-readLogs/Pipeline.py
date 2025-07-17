import os
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.gridspec import GridSpec
from collections import defaultdict

from frameAggregator import FrameAggregator
from kalmanFilter import KalmanFilter
import selfSpeedEstimator
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
DEBUG_CLUSTER           = False  # Set to False to disable debug logs
DEBUG_VXVY              = False
DEBUG_MISSING_CLUSTER   = False

# -------------------------------------------------------------
# CONSTANTS: Simulation parameters
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES    = 10
FILTER_SNR_MIN                      = 12
FILTER_Z_MIN                        = -2
FILTER_Z_MAX                        = 2
FILTER_Y_MIN                        = 0.6
FILTER_Y_MAX                        = 15
FILTER_PHI_MIN                      = -85
FILTER_PHI_MAX                      = 85
FILTER_DOPPLER_MIN                  = 0.01
FILTER_DOPPLER_MAX                  = 8.0
CLUSTER_HISTORY_LENGTH              = 2  

#Defining the self-speed's Kalman filter process variance and measurement variance
KALMAN_FILTER_PROCESS_VARIANCE = 0.05
KALMAN_FILTER_MEASUREMENT_VARIANCE = 0.1

# -------------------------------------------------------------
# OBJECTS: Processing stages
# -------------------------------------------------------------
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=2)
grid_processor           = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)
cluster_tracks          = defaultdict(list) # Holds tracked clusters: dict of { persistent_id : [centroid_history] }
kalmanRadar = KalmanFilter(process_variance=KALMAN_FILTER_PROCESS_VARIANCE, measurement_variance=KALMAN_FILTER_MEASUREMENT_VARIANCE)
kalmanIMU = KalmanFilter(process_variance=KALMAN_FILTER_PROCESS_VARIANCE, measurement_variance=KALMAN_FILTER_MEASUREMENT_VARIANCE)

# ---------------------------------------------------------------------------
# GLOBALS: Backups and flags for cluster gap estimation
# ---------------------------------------------------------------------------
prev_cluster_snapshots = {}  # Stores last known clusters for each persistent ID
missing_clusters = []        # Stores IDs missing in current frame for gap fill
# Global variable to hold clusters per frame
prev_frame_clusters = []
current_frame_clusters = []
missing_cluster_snapshots = {}
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


# ---------------------------------------------------------------------------
# GLOBALS: Backups and flags for cluster gap estimation
# ---------------------------------------------------------------------------
prev_cluster_snapshots = {}  # Stores last known clusters for each persistent ID
missing_clusters = []        # Stores IDs missing in current frame for gap fill

# ---------------------------------------------------------------------------
# FUNCTION: update_sim
# PURPOSE:
#   - Processes new frames
#   - Tracks persistent IDs using nearest neighbor
#   - Updates backup snapshot
#   - Detects missing clusters and flags them for estimation
# ---------------------------------------------------------------------------
def update_sim(new_frame):
    global curr_num_frame
    global prev_frame_clusters
    global prev_cluster_snapshots
    global missing_clusters
    global missing_cluster_snapshots

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
        cluster_pointCloud_stage1, _ = cluster_processor_stage1.cluster_points(points_for_clustering)

        # ------------------------
        # STAGE 2 CLUSTERING
        # ------------------------
        if len(cluster_pointCloud_stage1) > 0:
            points_stage1_flat = np.vstack([cdata['points'] for cdata in cluster_pointCloud_stage1.values()])
            cluster_pointCloud_stage2, _ = cluster_processor_stage2.cluster_points(points_stage1_flat)

            for cid, cdata in cluster_pointCloud_stage2.items():
                points = cdata['points'] # shape (N, 4): [x,y,z,doppler]
                x = points[:, 0]
                y = points[:, 1]
                """
                Calculation to obtain the azimuth angle (phi) in radians from the radar to the detected target.
                The azimuth angle is calculated using the arctangent of the y and x coordinates.
                So you get each point's bearing in the 2D plane.
                    -Angles is computed point by point.
                """
                phis = np.arctan2(x, y) # Y is Forward Heading → atan2(x, y)
                dopplers = points[:, 3] # Doppler speed of each point in the cluster
                """
                A is a matrix of unit vectors along each Line of Sight (LOS) to the target.
                Each row corresponds to a point in the cluster, with the first column being cos(phi)
                """
                A = np.stack([np.cos(phis), np.sin(phis)], axis=1)
                R = dopplers.reshape(-1, 1)
                # Residuals -> Should be close to 0 for rigid, coherent clusters Rank ->  number of linearly independent rows/columns in A
                V, residuals, rank, s = np.linalg.lstsq(A, R, rcond=None)
                v_x, v_y = V.flatten()

                # ------------------------
                # Validation of prediction of measurements for validation purposes
                # ------------------------
                predicted_R = v_x * np.cos(phis) + v_y * np.sin(phis)
                mean_measured = np.mean(dopplers)
                mean_predicted = np.mean(predicted_R)
                rms_error = np.sqrt(np.mean((dopplers - predicted_R)**2))

                if DEBUG_VXVY:
                    speed_mag = np.sqrt(v_x**2 + v_y**2)
                    direction_rad = np.arctan2(v_y, v_x)
                    direction_deg = (np.degrees(direction_rad) + 360) % 360

                    print("==========================================================")
                    print(f"[DEBUG_VXVY] Cluster {cid}")
                    print(f"→ Mean Cartesian Speed Vector: Vx = {v_x:.2f} m/s, Vy = {v_y:.2f} m/s")
                    print(f"→ Speed Magnitude: {speed_mag:.2f} m/s")
                    print(f"→ Direction (angle from X+ axis): {direction_deg:.2f}°")
                    print("----------------------------------------------------------")
                    print(f"→ Doppler Measured (mean):   {mean_measured:.2f} m/s")
                    print(f"→ Doppler Predicted (mean):  {mean_predicted:.2f} m/s")
                    print(f"→ Doppler RMS Error:         {rms_error:.4f} m/s")

                    doppler_sign_mismatch = np.sign(mean_measured) != np.sign(mean_predicted)
                    if doppler_sign_mismatch:
                        print(f"[DEBUG_VXVY] Doppler sign mismatch → possible velocity ambiguity or cluster error")
                    else:
                        print(f"[DEBUG_VXVY] Doppler signs agree → direction likely valid")
                    print("==========================================================")

                cdata['mean_vx'] = v_x
                cdata['mean_vy'] = v_y

            final_clusters = cluster_pointCloud_stage2
        else:
            final_clusters = {}

    # ------------------------
    # ASSOCIATION & TRACKING
    # ------------------------
    current_frame_clusters.clear()
    active_ids = set()
    if not prev_frame_clusters:
        next_id = 0
    else:
        next_id = max(prev['persistent_id'] for prev in prev_frame_clusters) + 1

    for cid, cdata in final_clusters.items():
        centroid = cdata['centroid']
        matched_id = None
        min_dist = np.inf

        for prev in prev_frame_clusters:
            dist = np.linalg.norm(centroid - prev['centroid'])
            if dist < 1.0 and dist < min_dist:
                matched_id = prev['persistent_id']
                min_dist = dist

        if matched_id is not None:
            persistent_id = matched_id
        else:
            persistent_id = next_id
            next_id += 1

        # Clear from multi-frame persistence if re-found
        if persistent_id in missing_cluster_snapshots:
            del missing_cluster_snapshots[persistent_id]

        cdata['persistent_id'] = persistent_id
        cluster_tracks[persistent_id].append(centroid)
        if len(cluster_tracks[persistent_id]) > CLUSTER_HISTORY_LENGTH:
            cluster_tracks[persistent_id].pop(0)

        # Backup for gap fill
        prev_cluster_snapshots[persistent_id] = {
            'persistent_id': persistent_id,
            'centroid': centroid,
            'points': cdata['points'],
            'priority': cdata['priority'],
            'doppler_avg': cdata['doppler_avg'],
            'mean_vx': cdata.get('mean_vx', 0.0),
            'mean_vy': cdata.get('mean_vy', 0.0)
        }

        active_ids.add(persistent_id)
        current_frame_clusters.append(prev_cluster_snapshots[persistent_id])

    # ------------------------
    # GAP CHECK: Mark missing IDs for single-frame or multi-frame fill
    # ------------------------
    missing_clusters.clear()
    for pid in prev_cluster_snapshots.keys():
        if pid not in active_ids:
            missing_clusters.append(pid)
            if pid not in missing_cluster_snapshots:
                # Store snapshot only once when first missing
                missing_cluster_snapshots[pid] = prev_cluster_snapshots[pid].copy()
            centroid = prev_cluster_snapshots[pid]['centroid']
            if DEBUG_MISSING_CLUSTER:
                print(f"[DEBUG] Frame {new_frame} | Cluster ID {pid} missing | Last centroid = {centroid}")

    # Clean up tracks that are neither active nor persistent
    for pid in list(cluster_tracks.keys()):
        if pid not in active_ids and pid not in missing_clusters and pid not in missing_cluster_snapshots:
            del cluster_tracks[pid]

    prev_frame_clusters = current_frame_clusters.copy()

    # ------------------------
    # Self-Speed Estimation
    # ------------------------
    Ve = selfSpeedEstimator.estimate_self_speed(filtered_pointCloud)
    Ve_filtered = kalmanRadar.update(Ve)

    accel_x = imu_avg['free_accel_x']
    accel_y = imu_avg['free_accel_y']
    Ve_imu = np.sqrt(accel_x**2 + accel_y**2)
    Ve_imu_filtered = kalmanIMU.update(Ve_imu)

    update_graphs(
        raw_var=raw_pointCloud,
        filtered_var=filtered_pointCloud,
        cluster_var=final_clusters,
        imu_var=imu_avg
    )

    print("-----------------------------------------------------------------")
    print("[Pipeline] Frame {} Estimated Self Speed = {:.2f} m/s".format(new_frame, Ve))
    print("[Pipeline] Frame {} Kalman Self Speed    = {:.2f} m/s".format(new_frame, Ve_filtered))
    print("[Pipeline] Frame {} Kalman IMU Speed     = {:.2f} m/s".format(new_frame, Ve_imu_filtered))

    curr_num_frame = new_frame

# ---------------------------------------------------------------------------
# FUNCTION: update_graphs
# PURPOSE:
#   - Handles normal cluster plotting.
#   - Adds gap fill for clusters missing in current frame.
# ---------------------------------------------------------------------------
def update_graphs(raw_var, filtered_var, cluster_var, imu_var):
    ax_estimation = axes['Estimated-Clusters']
    ax_estimation.clear()
    ax_estimation.set_title('Estimated Clusters')
    ax_estimation.set_xlim(-10, 10)
    ax_estimation.set_ylim(0, 15)
    ax_estimation.set_zlim(-2, 10)
    ax_estimation.view_init(elev=90, azim=-90)

    MMWAVE_FPS = 30
    DT = FRAME_AGGREGATOR_NUM_PAST_FRAMES / MMWAVE_FPS
    legend_added = {'original': False, 'estimated': False}

    # ------------------------
    # NORMAL ESTIMATES
    # ------------------------
    if cluster_var:
        for _, cluster_data in cluster_var.items():
            points = cluster_data['points']
            centroid = cluster_data['centroid']
            vx = cluster_data.get('mean_vx', 0.0)
            vy = cluster_data.get('mean_vy', 0.0)

            pred_points = np.copy(points)
            pred_points[:, 0] += vx * DT
            pred_points[:, 1] += vy * DT

            ax_estimation.scatter(
                points[:, 0], points[:, 1], points[:, 2],
                c='blue', s=8, alpha=0.4,
                label='Current Cluster' if not legend_added['original'] else ""
            )
            legend_added['original'] = True

            ax_estimation.scatter(
                pred_points[:, 0], pred_points[:, 1], pred_points[:, 2],
                c='lime', s=8, alpha=0.7,
                label='Estimated Next' if not legend_added['estimated'] else ""
            )
            legend_added['estimated'] = True

    # ------------------------
    # GAP-FILL ESTIMATES
    # ------------------------
    for pid in missing_clusters:
        snap = prev_cluster_snapshots[pid]
        centroid = snap['centroid']
        vx = snap['mean_vx']
        vy = snap['mean_vy']

        pred_points = np.copy(snap['points'])
        pred_points[:, 0] += vx * DT
        pred_points[:, 1] += vy * DT

        ax_estimation.scatter(
            pred_points[:, 0], pred_points[:, 1], pred_points[:, 2],
            c='red', s=8, alpha=0.7,
            label=f'Gap-Fill ID:{pid} \nVx={vx:.2f}, Vy={vy:.2f}"'
        )
        ax_estimation.text(
            centroid[0] + vx * DT, centroid[1] + vy * DT, centroid[2] + 0.1,
            f"ID:{pid} (Gap)\nVx={vx:.2f}, Vy={vy:.2f}",
            fontsize=7, 
            color='purple'
        )

    # ------------------------
    # PERSISTED GAP-FILL ESTIMATES (multi-frame)
    # ------------------------
    for pid, snap in missing_cluster_snapshots.items():
        centroid = snap['centroid']
        vx = snap['mean_vx']
        vy = snap['mean_vy']

        pred_points = np.copy(snap['points'])
        pred_points[:, 0] += vx * DT
        pred_points[:, 1] += vy * DT

        ax_estimation.scatter(
            pred_points[:, 0], pred_points[:, 1], pred_points[:, 2],
            c='magenta', s=8, alpha=0.7,
            label=f'Persist ID:{pid}'
        )
        ax_estimation.text(
            centroid[0] + vx * DT, centroid[1] + vy * DT, centroid[2] + 0.1,
            f"ID:{pid} (Persist)\nVx={vx:.2f}, Vy={vy:.2f}",
            fontsize=7, 
            color='purple'
        )

        # Update snapshot position for next estimate
        snap['centroid'] = estimated_next_position(centroid, vx, vy, DT)
        snap['points'][:, 0] += vx * DT
        snap['points'][:, 1] += vy * DT

    handles, labels = ax_estimation.get_legend_handles_labels()
    if handles:
        ax_estimation.legend(loc='upper right', fontsize=7)
        
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
                color='purple',
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
                        f"{pt[3]:.2f} m/s", 
                        fontsize=7,
                        color='purple')
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
                f"ID:{cluster_data['persistent_id']} P:{priority} {doppler_avg:.2f} m/s",
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

    # Estimate linear velocity from forward acceleration (assuming +Y body axis)
    accel_x = imu_var.get('free_accel_x', 0.0)
    accel_y = imu_var.get('free_accel_y', 0.0)
    accel_imu = np.sqrt(accel_x**2 + accel_y**2)
    frame_linear_velocity = kalmanIMU.update(accel_imu)

    ax_dir = axes['IMU-Direction']
    ax_dir.clear()
    ax_dir.set_title(
        f"IMU 2D Heading: {math.degrees(_compute_yaw(imu_var)):.1f}° \n "
        f"Estimated Linear Velocity: {frame_linear_velocity:.2f} m/s"
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
                f"ID:{cluster_data['persistent_id']} (Gap-Fill)\nVx={vx:.2f} Vy={vy:.2f}",
                fontsize=7, color='purple'
            )

    # Add legend
    handles, labels = ax_estimation.get_legend_handles_labels()
    if handles:
        ax_estimation.legend(loc='upper right', fontsize=7)


    # -------------------------------------------------------------
    # PLOT: Cluster Trajectories 
    # -------------------------------------------------------------
    axes['Cluster-Trajectories'].clear()
    axes['Cluster-Trajectories'].set_title("Cluster Trajectories")
    axes['Cluster-Trajectories'].set_xlabel("X [m]")
    axes['Cluster-Trajectories'].set_ylabel("Y [m]")
    axes['Cluster-Trajectories'].set_zlabel("Z [m]")
    axes['Cluster-Trajectories'].set_xlim(-10, 10)
    axes['Cluster-Trajectories'].set_ylim(0, 15)
    axes['Cluster-Trajectories'].set_zlim(-2, 10)
    axes['Cluster-Trajectories'].view_init(elev=90, azim=-90)

    # Plot each tracked cluster using its persistent ID
    for pid, history in cluster_tracks.items():
        history = np.array(history)
        axes['Cluster-Trajectories'].plot(
            history[:, 0], history[:, 1], history[:, 2],
            marker='o', label=f"ID:{pid}"
        )

        # Optionally, draw a simple extension line to show heading
        if len(history) >= 2:
            last = history[-1]
            prev = history[-2]

            dx = last[0] - prev[0]
            dy = last[1] - prev[1]

            norm = np.hypot(dx, dy) + 1e-6  # Avoid div by zero
            dx_unit = dx / norm
            dy_unit = dy / norm

            ext_length = 2.0  # meters for visual clarity

            axes['Cluster-Trajectories'].quiver(
                last[0], last[1], last[2],
                dx_unit, dy_unit, 0,
                length=ext_length,
                normalize=False,
                color='blue',
                arrow_length_ratio=0.2
            )

            axes['Cluster-Trajectories'].text(
                last[0] + dx_unit * ext_length,
                last[1] + dy_unit * ext_length,
                last[2] + 0.1,
                f"m={dy/dx:.2f}" if abs(dx) > 1e-3 else "m=inf",
                fontsize=7,
                color='blue'
            )

    handles, labels = axes['Cluster-Trajectories'].get_legend_handles_labels()
    if handles:
        axes['Cluster-Trajectories'].legend(fontsize=7)


           

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
    (2, 3),
    names=["Raw-PointCloud", "Filter-PointCloud", "DB-Clustered-PointCloud", "Estimated-Clusters", "IMU-Direction", "Cluster-Trajectories"],
    projections=["3d", "3d", "3d", "3d", "2d", "3d"]
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
