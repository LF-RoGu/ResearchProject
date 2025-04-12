import os
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
import occupancyGrid
from fileSearch import find_project_root


# -------------------------------
# Simulation parameters
# -------------------------------
#Defining the number of how many frames from the past should be used in the frame aggregator
# 0 = only current frame
# n = current frame + n previous frames
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 9

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

#Define grid
grid_processor = occupancyGrid.OccupancyGridProcessor(grid_spacing=0.5)



# -------------------------------
# FUNCTION: Updating the simulation when the value of the slider has changed
# -------------------------------
self_speed_raw_history = []
self_speed_filtered_history = []
def update_sim(new_num_frame):
    global curr_num_frame
    global self_speed_raw_history
    global self_speed_filtered_history
    
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
    
    #Simulating the necessary frames
    for num_frame in range(curr_num_frame + 1, new_num_frame + 1, 1):
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
        point_cloud_clustered = clusters_stage2

        ##Feeding the histories for the self speed
        self_speed_raw_history.append(self_speed_raw)
        self_speed_filtered_history.append(self_speed_filtered)
        

    #Updating the graphs
    update_graphs(point_cloud, point_cloud_filtered, point_cloud_ve_filtered, self_speed_raw_history, self_speed_filtered_history, point_cloud_clustered)

    #Updating the current frame number to the new last processed frame
    curr_num_frame = new_num_frame



# -------------------------------
# FUNCTION: Updating the simulation's graphs
# -------------------------------
def update_graphs(raw_points, point_cloud_points, filtered_points, self_speed_raw_history, self_speed_filtered_history, cluster_points):
    global frames
    point_cloud_clustered = pointFilter.extract_points(cluster_points)
    
    ##Plotting the points in the 3D plot
    #Creating arrays of the x,y,z coordinates
    points_x = np.array([point["x"] for point in raw_points])
    points_y = np.array([point["y"] for point in raw_points])
    points_z = np.array([point["z"] for point in raw_points])

    plot_raw_data.clear()
    plot_raw_data.set_title('Raw Point Cloud - Raw')
    plot_raw_data.set_xlabel('X [m]')
    plot_raw_data.set_ylabel('Y [m]')
    plot_raw_data.set_zlabel('Z [m]')
    plot_raw_data.set_xlim(-10, 10)
    plot_raw_data.set_ylim(0, 15)
    plot_raw_data.set_zlim(-0.30, 10)
    plot_raw_data.scatter(points_x, points_y, points_z)

    points_x = np.array([point["x"] for point in point_cloud_points])
    points_y = np.array([point["y"] for point in point_cloud_points])
    points_z = np.array([point["z"] for point in point_cloud_points])

    plot_pointCloud_data.clear()
    plot_pointCloud_data.set_title('Point Cloud Filters - Physical Filters')
    plot_pointCloud_data.set_xlabel('X [m]')
    plot_pointCloud_data.set_ylabel('Y [m]')
    plot_pointCloud_data.set_zlabel('Z [m]')
    plot_pointCloud_data.set_xlim(-10, 10)
    plot_pointCloud_data.set_ylim(0, 15)
    plot_pointCloud_data.set_zlim(-0.30, 10)
    plot_pointCloud_data.scatter(points_x, points_y, points_z)

    #Creating arrays of the x,y,z coordinates
    points_x = np.array([point["x"] for point in filtered_points])
    points_y = np.array([point["y"] for point in filtered_points])
    points_z = np.array([point["z"] for point in filtered_points])

    #Clearing the plot and plotting the points in the 3D plot
    plot_filtered_data.clear()
    plot_filtered_data.set_title('Point Cloud Filters - Ve Filters')
    plot_filtered_data.set_xlabel('X [m]')
    plot_filtered_data.set_ylabel('Y [m]')
    plot_filtered_data.set_zlabel('Z [m]')
    plot_filtered_data.set_xlim(-10, 10)
    plot_filtered_data.set_ylim(0, 15)
    plot_filtered_data.set_zlim(-0.30, 10)
    plot_filtered_data.scatter(points_x, points_y, points_z)

    #Plotting the raw and filtered self-speed
    plot_Ve.clear()
    plot_Ve.set_title('Vehicle Ve')
    plot_Ve.set_xlim(0, len(frames))
    plot_Ve.set_ylim(-3, 0)
    plot_Ve.plot(np.arange(0, len(self_speed_raw_history)), np.array(self_speed_raw_history), linestyle='--')
    plot_Ve.plot(np.arange(0, len(self_speed_filtered_history)), np.array(self_speed_filtered_history))
    #plot_Ve.text(0.0, 0.0, f"Current Speed: {self_speed_filtered_history[-1]} m/s", color='purple')

    # -------------------------------
    # PLOT 3: Clustered Point Cloud (Top-Right)
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
    # PLOT 4: Occupancy Grid (Bottom-Right)
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
plot_raw_data =         fig.add_subplot(gs[0, 0], projection='3d')
plot_pointCloud_data =  fig.add_subplot(gs[0, 1], projection='3d')
plot_dbCluster =        fig.add_subplot(gs[0, 2], projection='3d')
plot_filtered_data =    fig.add_subplot(gs[1, 0], projection='3d')
plot_Ve =               fig.add_subplot(gs[1, 1])
plot_occupancyGrid =    fig.add_subplot(gs[1, 2])

#Setting the initial view angle of the 3D-plot to top-down
plot_raw_data.view_init(elev=90, azim=-90)
plot_pointCloud_data.view_init(elev=90, azim=-90)
plot_filtered_data.view_init(elev=90, azim=-90)
plot_dbCluster.view_init(elev=90, azim=-90)

#Variable to hold the number of the latest frame that was processed successfully
curr_num_frame = -1

#Creating a slider for frame selection and attaching a handler to the on_changed event
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03])
slider = Slider(ax_slider, 'Frame', 0, len(frames) - 1, valinit=0, valstep=1)
slider.on_changed(update_sim)

##Starting the simulation with the first frame and showing the plot
update_sim(0)
plt.show()