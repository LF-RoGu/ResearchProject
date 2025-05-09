from cmath import sin
import os
from matplotlib.pylab import cos
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.gridspec import GridSpec

import dataDecoder
from frameAggregator import FrameAggregator
import pointFilter
import selfSpeedEstimator
from kalmanFilter import KalmanFilter
import veSpeedFilter
import dbCluster
from dbCluster import compress_cluster_to_point
import occupancyGrid
from fileSearch import find_project_root


# -------------------------------
# Simulation parameters
# -------------------------------
#Defining the number of how many frames from the past should be used in the frame aggregator
# 0 = only current frame
# n = current frame + n previous frames
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 5

#Defining a minimum SNR for the filter stage
FILTER_SNR_MIN = 12

#Defining minimum and maximum z for the filter stage
FILTER_Z_MIN = -0.3
FILTER_Z_MAX = 2

#Defining minimum and maximum phi for the filter stage
FILTER_PHI_MIN = -85
FILTER_PHI_MAX = 85

#Defining the self-speed's Kalman filter process variance and measurement variance
KALMAN_FILTER_PROCESS_VARIANCE = 0.01
KALMAN_FILTER_MEASUREMENT_VARIANCE = 0.1

#Defining dbClustering stages
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=4)
#VxVy Processing
VxVy_processor_stage = dbCluster.ClusterProcessor(eps=0.2, min_samples=2)

#Define grid
grid_processor = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)



# -------------------------------
# FUNCTION: Updating the simulation when the value of the slider has changed
# -------------------------------
self_speed_raw_history = []
self_speed_filtered_history = []
prev_point_cloud_clustered = []
prev_clusters_compressed = []

def update_sim(new_num_frame):
    global curr_num_frame
    global self_speed_raw_history
    global self_speed_filtered_history
    global prev_point_cloud_clustered
    global prev_clusters_compressed
    
    #Checking if new frame is earlier than the current processed frame (--> simulation needs to be rebuild until this particular frame)
    if new_num_frame < curr_num_frame:
            ##Clearing the pipeline
            #Clearing the frame aggregator
            frame_aggregator.clearBuffer()

            #Resetting the Kalman filter
            self_speed_kf.clear()
            

            ##Clearing the history variables
            self_speed_raw_history.clear()
            self_speed_filtered_history.clear()

            #Setting the current frame to -1 to start feeding at index 0
            curr_num_frame = -1
            # Reset variable when the simulation gets updated.
            prev_point_cloud_clustered = None
            prev_clusters_compressed = []
    
    #Simulating the necessary frames
    for num_frame in range(curr_num_frame + 1, new_num_frame + 1, 1):
        cluster_results_vxvy = None
        ##Feeding the pipeline
        #Getting the current frame
        frame = frames[num_frame]

        #Updating the frame aggregator
        frame_aggregator.updateBuffer(frame)

        #Getting the current point cloud frum the frame aggregator
        point_cloud = frame_aggregator.getPoints()

        #Filtering by z
        point_cloud_filtered = pointFilter.filterCartesianZ(point_cloud, FILTER_Z_MIN, FILTER_Z_MAX)

        #Filtering by phi
        point_cloud_filtered = pointFilter.filterSphericalPhi(point_cloud_filtered, FILTER_PHI_MIN, FILTER_PHI_MAX)

        #Estimating the self-speed
        self_speed_raw = selfSpeedEstimator.estimate_self_speed(point_cloud_filtered)

        #Kalman filtering the self-speed
        self_speed_filtered = self_speed_kf.update(self_speed_raw)

        #Filtering point cloud by Ve
        point_cloud_ve_filtered = pointFilter.filter_by_speed(point_cloud_filtered, self_speed_filtered, 1.0)

        # -------------------------------
        # STEP 1: First Clustering Stage
        # -------------------------------
        point_cloud_clustering_stage1 = pointFilter.extract_points(point_cloud_ve_filtered)
        clusters_stage1, _ = cluster_processor_stage1.cluster_points(point_cloud_clustering_stage1)
        point_cloud_clustering_stage2 = pointFilter.extract_points(clusters_stage1)
        clusters_stage2, _ = cluster_processor_stage2.cluster_points(point_cloud_clustering_stage2)

        # Final cluster step
        #point_cloud_clustered = pointFilter.extract_points(clusters_stage2)
        curr_point_cloud_clustered = clusters_stage2

        # -------------------------------
        # Vx and Vy calculation
        # -------------------------------
        aggregated_compressed_cluster_points = []
        cluster_aggregated_compressed_cluster_points = []
        #Compress each cluster into a single representative point
        curr_clusters_compressed = [
                                    compress_cluster_to_point(cluster_data)
                                    for cluster_data in curr_point_cloud_clustered.values()
                                    ]

        if prev_clusters_compressed:
            num_current = len(curr_clusters_compressed)
            num_previous = len(prev_clusters_compressed)

            # Add all previous frame compressed clusters
            for cluster in prev_clusters_compressed:
                if cluster:
                    aggregated_compressed_cluster_points.append({
                        'x': cluster['x'],
                        'y': cluster['y'],
                        'z': cluster['z'],
                        'doppler': cluster['doppler'],
                        'frame': 't-1'
                    })
                    cluster_aggregated_compressed_cluster_points.append({
                        'x': cluster['x'],
                        'y': cluster['y'],
                        'z': cluster['z'],
                        'doppler': cluster['doppler'],
                    })

            # Add all current frame compressed clusters
            for cluster in curr_clusters_compressed:
                if cluster:
                    aggregated_compressed_cluster_points.append({
                        'x': cluster['x'],
                        'y': cluster['y'],
                        'z': cluster['z'],
                        'doppler': cluster['doppler'],
                        'frame': 't'
                    })
                    cluster_aggregated_compressed_cluster_points.append({
                        'x': cluster['x'],
                        'y': cluster['y'],
                        'z': cluster['z'],
                        'doppler': cluster['doppler'],
                    })
            # [!] Variable to indicate the end of calculation for compressed clusters
            cluster_ac_points = pointFilter.extract_points(cluster_aggregated_compressed_cluster_points)
            cluster_aggregated_compressed_cluster_points, _ = VxVy_processor_stage.cluster_points(cluster_ac_points)

            cluster_results_vxvy = {}
            for cluster_id, cluster_data in cluster_aggregated_compressed_cluster_points.items():

                points = cluster_data['points']
                if len(points) != 2:
                    print(f"Skipping cluster {cluster_id}: requires exactly 2 points (found {len(points)})")
                    continue

                # Extract points (assumed one from t-1, one from t)
                p1, p2 = points[0], points[1]
                x1, y1, _, doppler1 = p1
                x2, y2, _, doppler2 = p2

                # Calculate angles from radar origin
                phi1 = np.arctan2(y1, x1)
                phi2 = np.arctan2(y2, x2)

                # Build system of equations:
                # doppler = vx * cos(phi) + vy * sin(phi)
                A = np.array([
                    [np.cos(phi1), np.sin(phi1)],
                    [np.cos(phi2), np.sin(phi2)]
                ])
                b = np.array([doppler1, doppler2])

                try:
                    """
                    Detect identical points and skip them.
                    Both rows are identical.
                    It's underdetermined.
                    This means your matrix A is rank 1 (not invertible).
                    """
                    if np.allclose(p1, p2, atol=1e-4):
                        print(f"Cluster {cluster_id}: skipped because both points are identical.")
                        continue

                    vx, vy = np.linalg.solve(A, b)
                    cluster_results_vxvy[cluster_id] = {
                        'points': points,
                        'centroid': cluster_data['centroid'],
                        'priority': cluster_data['priority'],
                        'doppler_avg': cluster_data['doppler_avg'],
                        'vx': vx,
                        'vy': vy
                    }

                    print(f"\nCluster {cluster_id}:")
                    for point in points:
                        print(f"  (x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}, doppler={point[3]:.2f})")
                    print(f"  vx = {vx:.2f} m/s, vy = {vy:.2f} m/s")
                except np.linalg.LinAlgError:
                    print(f"\nCluster {cluster_id}: Cannot solve, matrix is singular or ill-conditioned.")
                    print("  Points used for this cluster:")
                    for point in cluster_data['points']:
                        print(f"    (x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}, doppler={point[3]:.2f})")
    


        prev_point_cloud_clustered = curr_point_cloud_clustered
        prev_clusters_compressed = curr_clusters_compressed

        ##Feeding the histories for the self speed
        self_speed_raw_history.append(self_speed_raw)
        self_speed_filtered_history.append(self_speed_filtered)
        
    #Updating the graphs
    update_graphs(self_speed_raw_history, self_speed_filtered_history, curr_point_cloud_clustered, aggregated_compressed_cluster_points, cluster_aggregated_compressed_cluster_points, cluster_results_vxvy)

    #Updating the current frame number to the new last processed frame
    curr_num_frame = new_num_frame



# -------------------------------
# FUNCTION: Updating the simulation's graphs
# -------------------------------
def update_graphs(self_speed_raw_history, self_speed_filtered_history, cluster_points, compressed_cluster_points, aggregated_compressed_cluster_points, cluster_results_vxvy):
    global frames
    point_cloud_clustered = pointFilter.extract_points(cluster_points)

    #Plotting the raw and filtered self-speed
    plot_Ve.clear()
    plot_Ve.set_title('Vehicle Ve')
    plot_Ve.set_xlim(0, len(frames))
    plot_Ve.set_ylim(-3, 0)
    plot_Ve.plot(np.arange(0, len(self_speed_raw_history)), np.array(self_speed_raw_history), linestyle='--')
    plot_Ve.plot(np.arange(0, len(self_speed_filtered_history)), np.array(self_speed_filtered_history))
    #plot_Ve.text(0.0, 0.0, f"Current Speed: {self_speed_filtered_history[-1]} m/s", color='purple')

    # -------------------------------
    # PLOT: Clustered Point Cloud
    # -------------------------------
    priority_colors = {1: 'red', 2: 'orange', 3: 'green'}

    plot_dbCluster.clear()
    plot_dbCluster.set_title('Clustered Point Cloud')
    plot_dbCluster.set_xlabel('X [m]')
    plot_dbCluster.set_ylabel('Y [m]')
    plot_dbCluster.set_zlabel('Z [m]')
    plot_dbCluster.set_xlim(-10, 10)
    plot_dbCluster.set_ylim(0, 15)
    plot_dbCluster.set_zlim(-0.30, 10)
    if cluster_points:
        for _, cluster_data in cluster_points.items():
            centroid = cluster_data['centroid']
            priority = cluster_data['priority']
            points = cluster_data['points']
            doppler_avg = cluster_data['doppler_avg']  # Access average Doppler
            
            color = priority_colors.get(priority, 'gray')  # Default to gray if priority not in the dictionary

            # Plot the cluster points
            plot_dbCluster.scatter(points[:, 0], points[:, 1], points[:, 2], c=color, s=8, alpha=0.7, label=f'Priority {priority}')

            # Add Doppler and Priority labels at the centroid
            plot_dbCluster.text(centroid[0] + 0.2, centroid[1] + 0.2, centroid[2] + 0.2, f"{doppler_avg:.2f} m/s", color='purple')
    else:
        plot_dbCluster.text(0, 0, 0, 'No Clusters Detected', fontsize=12, color='red')

    # -------------------------------
    # PLOT: Compressed Clustered Point Cloud
    # -------------------------------
    priority_colors = {1: 'red', 2: 'orange', 3: 'green'}

    plot_compressedDB_data.clear()
    plot_compressedDB_data.set_title('Compressed Clustered Point Cloud')
    plot_compressedDB_data.set_xlabel('X [m]')
    plot_compressedDB_data.set_ylabel('Y [m]')
    plot_compressedDB_data.set_zlabel('Z [m]')
    plot_compressedDB_data.set_xlim(-10, 10)
    plot_compressedDB_data.set_ylim(0, 15)
    plot_compressedDB_data.set_zlim(-0.30, 10)
    if aggregated_compressed_cluster_points:
        for _, cluster_data in aggregated_compressed_cluster_points.items():
            centroid = cluster_data['centroid']
            priority = cluster_data['priority']
            points = cluster_data['points']
            doppler_avg = cluster_data['doppler_avg']  # Access average Doppler
            
            color = priority_colors.get(priority, 'gray')  # Default to gray if priority not in the dictionary

            # Plot the cluster points
            plot_compressedDB_data.scatter(points[:, 0], points[:, 1], points[:, 2], c=color, s=8, alpha=0.7, label=f'Priority {priority}')

            # Add Doppler and Priority labels at the centroid
            plot_compressedDB_data.text(centroid[0] + 0.2, centroid[1] + 0.2, centroid[2] + 0.2, f"{doppler_avg:.2f} m/s", color='purple')
    else:
        plot_compressedDB_data.text(0, 0, 0, 'No Clusters Detected', fontsize=12, color='red')

    # -------------------------------
    # PLOT: Compressed Clustered Point Cloud Vx and Vy
    # -------------------------------
    priority_colors = {1: 'red', 2: 'orange', 3: 'green'}

    plot_vxvy_data.clear()
    plot_vxvy_data.set_title('Compressed Clustered Point Cloud')
    plot_vxvy_data.set_xlabel('X [m]')
    plot_vxvy_data.set_ylabel('Y [m]')
    plot_vxvy_data.set_zlabel('Z [m]')
    plot_vxvy_data.set_xlim(-10, 10)
    plot_vxvy_data.set_ylim(0, 15)
    plot_vxvy_data.set_zlim(-0.30, 10)

    if cluster_results_vxvy:
        for cluster_id, cluster_data in cluster_results_vxvy.items():
            centroid = cluster_data['centroid']
            priority = cluster_data['priority']
            points = cluster_data['points']
            doppler_avg = cluster_data['doppler_avg']
            vx = cluster_data['vx']
            vy = cluster_data['vy']

            # Choose color based on priority
            color = priority_colors.get(priority, 'gray')

            # Plot the cluster points
            plot_vxvy_data.scatter(points[:, 0], points[:, 1], points[:, 2],
                                c=color, s=8, alpha=0.7, label=f'Priority {priority}')

            # Add velocity vector as an arrow from the centroid
            plot_vxvy_data.quiver(
                centroid[0], centroid[1], centroid[2],  # Origin of arrow
                vx, vy, 0,                              # Vx/Vy only in XY plane
                length=1.0, color='blue', normalize=True
            )

            # Label with Doppler average
            plot_vxvy_data.text(centroid[0] + 0.2, centroid[1] + 0.2, centroid[2] + 0.2,
                                f"{doppler_avg:.2f} m/s", color='purple')
    else:
        plot_vxvy_data.text(0, 0, 0, 'No Clusters Detected', fontsize=12, color='red')

    # -------------------------------
    # PLOT: Compressed Point Cloud
    # -------------------------------
    plot_compressed_data.clear()
    #Creating arrays of the x,y,z coordinates
    # Separate points from t-1 and t
    points_t_minus_1 = [p for p in compressed_cluster_points if p['frame'] == 't-1']
    points_t = [p for p in compressed_cluster_points if p['frame'] == 't']

    # Plot t-1 points in blue circles
    if points_t_minus_1:
        x1 = [p['x'] for p in points_t_minus_1]
        y1 = [p['y'] for p in points_t_minus_1]
        z1 = [p['z'] for p in points_t_minus_1]
        plot_compressed_data.scatter(x1, y1, z1, c='blue', marker='o', label='t-1')

    # Plot t points in red triangles
    if points_t:
        x2 = [p['x'] for p in points_t]
        y2 = [p['y'] for p in points_t]
        z2 = [p['z'] for p in points_t]
        plot_compressed_data.scatter(x2, y2, z2, c='red', marker='^', label='t')

    plot_compressed_data.set_title('Compressed Point Cloud - Compressed')
    plot_compressed_data.set_xlabel('X [m]')
    plot_compressed_data.set_ylabel('Y [m]')
    plot_compressed_data.set_zlabel('Z [m]')
    plot_compressed_data.set_xlim(-10, 10)
    plot_compressed_data.set_ylim(0, 15)
    plot_compressed_data.set_zlim(-0.30, 10)

    # -------------------------------
    # PLOT: Occupancy Grid
    # -------------------------------
    if point_cloud_clustered.size > 0:
        # Assuming grid_processor is initialized globally
        occupancy_grid = grid_processor.calculate_cartesian_grid(point_cloud_clustered[:, :2], x_limits=(-10, 10), y_limits=(0, 15))

        plot_occupancyGrid.clear()
        plot_occupancyGrid.set_title('Occupancy Grid')
        plot_occupancyGrid.set_xlabel('X [m]')
        plot_occupancyGrid.set_ylabel('Y [m]')
        plot_occupancyGrid.imshow(occupancy_grid.T, cmap=grid_processor.cmap, norm=grid_processor.norm, origin='lower', extent=(-10, 10, 0, 15))
    else:
        plot_occupancyGrid.clear()
        plot_occupancyGrid.set_title('Occupancy Grid (No Data)')



# -------------------------------
# Program entry point
# -------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = find_project_root(script_dir, "ResearchProject")
log_file = os.path.join(
    project_root, 
    "04_Logs", 
    "LogsPart3", 
    "DynamicMonitoring", 
    "Test_30fps_dist15mts_vehicleLog_5mps_3x3Wall_pedestrian_log.csv")
log_file = os.path.abspath(os.path.normpath(log_file))
frames = dataDecoder.decodeData(log_file)

#Creating the frame aggregator
frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

#Creating the Kalman filter for the self-speed esimation
self_speed_kf = KalmanFilter(process_variance=KALMAN_FILTER_PROCESS_VARIANCE, measurement_variance=KALMAN_FILTER_MEASUREMENT_VARIANCE)

##Setting up the visualization and starting the simulation
#Creating a figure of size 10x10
fig = plt.figure(figsize=(10, 10))

#Defining a 2x4 grid layout
gs = GridSpec(2, 3, figure=fig)
plot_dbCluster =        fig.add_subplot(gs[0, 0], projection='3d')
plot_compressed_data =  fig.add_subplot(gs[1, 1], projection='3d') # Temporal Plot
plot_compressedDB_data =  fig.add_subplot(gs[1, 2], projection='3d') # Temporal Plot
plot_vxvy_data =  fig.add_subplot(gs[0, 2], projection='3d') # Temporal Plot
plot_occupancyGrid =    fig.add_subplot(gs[0, 1])
plot_Ve =               fig.add_subplot(gs[1, 0])


#Setting the initial view angle of the 3D-plot to top-down
plot_dbCluster.view_init(elev=90, azim=-90)
plot_compressed_data.view_init(elev=90, azim=-90) # Temporal Plot
plot_compressedDB_data.view_init(elev=90, azim=-90) # Temporal Plot
plot_vxvy_data.view_init(elev=90, azim=-90) # Temporal Plot

#Variable to hold the number of the latest frame that was processed successfully
curr_num_frame = -1

#Creating a slider for frame selection and attaching a handler to the on_changed event
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(frames) - 1, valinit=0, valstep=1)
slider.on_changed(update_sim)

##Starting the simulation with the first frame and showing the plot
update_sim(0)
plt.show()