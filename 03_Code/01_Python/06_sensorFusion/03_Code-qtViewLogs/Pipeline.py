import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton
)
from PyQt5.QtCore import Qt

from ClusterTracker import ClusterTracker
from decodeFile import ImuCSVReader, RadarCSVReader
import helper
from frameAggregator import FrameAggregator
import dbCluster
import pointFilter
import icp
from kalmanFilter import KalmanFilter
from odoLog import logging
import selfSpeedEstimator

from ClusterTracker import (
    NUMBER_OF_PAST_SAMPLES,
    LINE_FIT_UPDATE_PERIOD,
    MAX_HISTORY_SEGMENT_LENGTH,
    LINE_DISTANCE_THRESHOLD,
    MAX_CONSECUTIVE_MISSES,
    ClusterTracker
)


# -------------------------------------------------------------
# PARAMETERS
# -------------------------------------------------------------

# 1 = Radar only, 2 = IMU only, 3 = Both
ENABLE_SENSORS = 3

FRAME_AGGREGATOR_NUM_PAST_FRAMES = 7
FILTER_SNR_MIN                  = 12
FILTER_Z_MIN, FILTER_Z_MAX      = 0, 4
FILTER_Y_MIN, FILTER_Y_MAX      = 0.6, 15
FILTER_PHI_MIN, FILTER_PHI_MAX  = -85, 85
FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX = 0.01, 8.0
TRACKER_MAX_MISSES  = 10     # how many frames a track can disappear before deletion
TRACKER_DIST_THRESH = 0.5   # max movement (meters) allowed per frame for matching

# Global variables for current (P) and previous (Q) frame clusters
P = None  # clusters for current frame t
Q = None  # clusters for previous frame t-1
P_global = None  # clusters for current frame t
Q_global = None  # clusters for previous frame t-1
egoMotion_global = {
    'translation':       [],  # Store cumulative global translations (x, y)
    'rotations':         []   # Store cumulative global heading (theta)
}
cumulative_distance_global = 0.0
cumulative_heading_global = 0.0
egoMotion_cluster = {
    'translation':       [],  # Store cumulative global translations (x, y)
    'rotations':         []   # Store cumulative global heading (theta)
}
cumulative_distance_cluster = 0.0
cumulative_heading_cluster = 0.0

trajectory_cluster = [(0.0, 0.0)]
trajectory_global = [(0.0, 0.0)]

trajectory_cluster_imu = [(0.0, 0.0)]
trajectory_global_imu = [(0.0, 0.0)]
imu_heading_rad = None

T_global = np.eye(3)  # initial pose at origin


folderName = "14_outside"  # Folder where CSV files are stored
testType = "outside7.csv"  # Type of test data
# Instantiate readers and global aggregators
radarLoaderA = RadarCSVReader("radarA_" + testType, folderName) if ENABLE_SENSORS in (1, 3) else None
radarLoaderB = RadarCSVReader("radarB_" + testType, folderName) if ENABLE_SENSORS in (1, 3) else None
imuLoader   = ImuCSVReader("imu_" + testType, folderName) if ENABLE_SENSORS in (2, 3) else None
_radarA_Agg         = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_radarB_Agg         = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_imuAgg           = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

# Two‐stage DBSCAN processors
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.5, min_samples=3)

Vego_filter = KalmanFilter(process_variance=0.02, measurement_variance=0.5)

def plot_trajectory(plot_widget, trajectory, label, color='g', extra_lines=None):
    plot_widget.clear()
    plot_widget.setTitle(label)
    plot_widget.showGrid(x=True, y=True)

    if len(trajectory) < 1:
        return

    # --- Plot main trajectory
    xs, ys = zip(*trajectory)
    plot_widget.plot(xs, ys, pen=pg.mkPen(color, width=2))
    last_x, last_y = xs[-1], ys[-1]
    label_item = pg.TextItem(f"{label}\nX={last_x:.2f}, Y={last_y:.2f}", color=color)
    label_item.setPos(last_x, last_y)
    plot_widget.addItem(label_item)

    # --- Plot extra lines if any
    if extra_lines:
        for extra in extra_lines:
            xs2, ys2 = zip(*extra['trajectory']) if len(extra['trajectory']) > 0 else ([], [])
            if xs2 and ys2:
                plot_widget.plot(xs2, ys2, pen=pg.mkPen(extra.get('color', 'r'), width=2, style=pg.QtCore.Qt.DashLine))
                last_x2, last_y2 = xs2[-1], ys2[-1]
                label_item2 = pg.TextItem(extra.get('label', ''), color=extra.get('color', 'r'))
                label_item2.setPos(last_x2, last_y2)
                plot_widget.addItem(label_item2)

def pretty_print_clusters(clusters, label="Clusters"):
    """
    Nicely prints a summary of `clusters` dict: ID, centroid, doppler, hits, and point count.
    """
    if clusters is None:
        print(f"{label}: None")
        return
    print(f"{label} ({len(clusters)} clusters):")
    for cid, data in clusters.items():
        centroid = data.get('centroid')
        dop = data.get('doppler_average')
        hits = data.get('hit_count', None)
        missed = data.get('miss_count', None)
        pts = data.get('points')
        pt_count = len(pts) if pts is not None else 0
        print(f" - ID {cid}: centroid={centroid}, doppler={dop:.2f}, hits={hits}, missed={missed}, points={pt_count}")

# ------------------------------
# Plot-1’s custom view (unchanged)
# ------------------------------
def plot1(plot_widget, pointCloud):
    """
    Render only the filtered point cloud (pointCloud),
    plotting Sensor A and Sensor B in different colors.
    """
    import pyqtgraph as pg

    # 1) Clear previous items and set title
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud - Filtered")

    # 2) Bail out if empty
    if not pointCloud:
        return

    # 3) Split points by sensor ID
    points_a = [p for p in pointCloud if p.get('sensorId') == 'A']
    points_b = [p for p in pointCloud if p.get('sensorId') == 'B']

    def extract_xy(points):
        return [p['x'] for p in points], [p['y'] for p in points]

    # 4) Plot Sensor A (green)
    x_a, y_a = extract_xy(points_a)
    scatter_a = pg.ScatterPlotItem(
        x=x_a,
        y=y_a,
        size=8,
        pen=None,
        brush=pg.mkBrush(0, 200, 0, 150)
    )
    plot_widget.addItem(scatter_a)

    # 5) Plot Sensor B (blue)
    x_b, y_b = extract_xy(points_b)
    scatter_b = pg.ScatterPlotItem(
        x=x_b,
        y=y_b,
        size=8,
        pen=None,
        brush=pg.mkBrush(0, 0, 255, 150)
    )
    plot_widget.addItem(scatter_b)

    # 6) Add Doppler text to all
    for p in pointCloud:
        if isinstance(p, dict) and 'x' in p and 'y' in p and 'doppler' in p:
            dop = p['doppler']
            txt = pg.TextItem(f"{dop:.2f}", color='y', anchor=(0, 1))
            txt.setPos(p['x'] + 0.1, p['y'] + 0.1)
            plot_widget.addItem(txt)



# ------------------------------
# Plot-2’s custom view (unchanged)
# ------------------------------
def plot2(plot_widget, clusters):
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud - Clustered")
    for cid,data in clusters.items():
        clusterPoints = data['points']
        clusterDoppler = data['doppler_average']
        clusterHits    = data.get('hit_count', 0)
        if clusterPoints.shape[0]>0:
            scatter = pg.ScatterPlotItem(
                x=clusterPoints[:,0], y=clusterPoints[:,1], size=8,
                pen=None, brush=pg.mkBrush(0,200,0,150)
            )
            plot_widget.addItem(scatter)
        # unpack centroid (x,y ignore others)
        cx, cy = data['centroid'][:2]
        label = pg.TextItem(
            f"ID: {cid}\n"
            f"Doppler: {clusterDoppler:.2f}\n"
            f"Hits: {clusterHits}\n"
            f"Cx,Cy: ({cx:.2f}, {cy:.2f})",
            anchor=(0.5, -0.2),
            color='w'
        )
        label.setPos(cx, cy)
        plot_widget.addItem(label)

# ---------------------------------
# Plot 3: Doppler vs Azimuth (RANSAC)
# ---------------------------------
def plot3(plot_widget, originalPoints, ransac_output):
    """
    RANSAC diagnostics: Doppler (m/s) vs Azimuth angle (deg).
    - Works with ransac_output where 'inliers' and 'outliers' are point lists (dicts).
    - Uses the same angle convention as the fitter: theta = atan2(y,x) - pi/2 (forward=0°).
    - Adds legend, fixes axes, disables auto range; robust to missing model/data.
    """
    import pyqtgraph as pg
    import numpy as np
    import math

    # Reset + fixed view (disable autofocus)
    plot_widget.clear()
    plot_widget.setTitle("RANSAC — Doppler vs Azimuth")
    plot_widget.setLabel('bottom', 'Azimuth θ (deg)')
    plot_widget.setLabel('left', 'Radial speed (m/s)')
    plot_widget.showGrid(x=True, y=True)

    # Keep exactly one legend (clear each frame)
    if getattr(plot_widget, "legend", None) is None:
        plot_widget.addLegend(offset=(10, 10))
    else:
        plot_widget.legend.clear()

    # Defensive unpack
    if not ransac_output or not isinstance(ransac_output, dict):
        note = pg.TextItem("No RANSAC data", color='r'); note.setPos(-170, 0)
        plot_widget.addItem(note)
        return

    model     = ransac_output.get("model", None)
    inliers   = ransac_output.get("inliers", [])   # list of dict points
    outliers  = ransac_output.get("outliers", [])  # list of dict points

    # Helper: compute theta_deg with same rotation used in fit: atan2(y,x) - pi/2
    def pts_to_theta_deg_and_doppler(pts):
        thetas_deg, dops = [], []
        for pt in pts:
            if not isinstance(pt, dict):  # safety
                continue
            if 'x' in pt and 'y' in pt and 'doppler' in pt:
                th = math.atan2(pt['y'], pt['x']) - math.pi/2
                # wrap to [-pi, pi] for nice plot continuity
                if th > math.pi:
                    th -= 2*math.pi
                elif th < -math.pi:
                    th += 2*math.pi
                thetas_deg.append(math.degrees(th))
                dops.append(pt['doppler'])
        return np.array(thetas_deg, dtype=float), np.array(dops, dtype=float)

    in_theta_deg, in_dop = pts_to_theta_deg_and_doppler(inliers)
    out_theta_deg, out_dop = pts_to_theta_deg_and_doppler(outliers)

    # If nothing to show, bail gracefully
    if in_theta_deg.size == 0 and out_theta_deg.size == 0:
        note = pg.TextItem("No points for RANSAC", color='r'); note.setPos(-170, 0)
        plot_widget.addItem(note)
        return

    # Plot inliers / outliers
    if in_theta_deg.size:
        plot_widget.plot(in_theta_deg, in_dop,
                         pen=None, symbol='o', symbolSize=7,
                         symbolBrush=(0, 255, 0, 140), name='Inliers')
    if out_theta_deg.size:
        plot_widget.plot(out_theta_deg, out_dop,
                         pen=None, symbol='x', symbolSize=8,
                         symbolBrush=(255, 0, 0, 160), name='Outliers')

    # Fitted curve (only if model exists and supports predict)
    if model is not None and hasattr(model, "predict"):
        try:
            theta_range_rad = np.linspace(-np.pi, np.pi, 300).reshape(-1, 1)
            doppler_fit     = model.predict(theta_range_rad)
            theta_range_deg = np.degrees(theta_range_rad).reshape(-1)
            plot_widget.plot(theta_range_deg, doppler_fit,
                             pen=pg.mkPen('y', width=2), name='RANSAC fit')
        except Exception:
            note = pg.TextItem("Fit unavailable (predict failed)", color='y'); note.setPos(-170, -1)
            plot_widget.addItem(note)
    else:
        note = pg.TextItem("Fit unavailable (no model)", color='y'); note.setPos(-170, -1)
        plot_widget.addItem(note)

# ------------------------------
# Plot-4’s custom view (unchanged)
# ------------------------------
def plot4(plot_widget, ego_matrix, translation_history):
    """
    Dedicated panel for visualizing ego-motion translation.
    - Plots 'x' marks showing translation over time
    - Uses the full 3x3 ego_matrix to extract position
    """
    plot_widget.clear()
    plot_widget.setTitle("Ego-Motion Translation")
    plot_widget.showGrid(x=True, y=True)

    if ego_matrix is None:
        txt = pg.TextItem("No Ego-Motion Data", color='r')
        txt.setPos(0, 0)
        plot_widget.addItem(txt)
        return

    # Extract the translation component from Tego
    tx = ego_matrix[0, 2]
    ty = ego_matrix[1, 2]

    # Store translation history
    translation_history.append((tx, ty))

    # Plot the history as 'x' marks
    if len(translation_history) > 0:
        xs, ys = zip(*translation_history)
        scatter = pg.ScatterPlotItem(
            x=xs, y=ys, symbol='x', size=10,
            pen=pg.mkPen('b', width=2)
        )
        plot_widget.addItem(scatter)

    # Show the latest translation value as text
    txt = pg.TextItem(f"Translation:\nX={tx:.2f}, Y={ty:.2f}", color='b')
    txt.setPos(tx, ty)
    plot_widget.addItem(txt)

# ------------------------------
# Plot-7’s 
# ------------------------------
def plot7(plot_widget, imu_average):
    """
    Display IMU heading on a unit circle, rotated so that
    0° = West, 90° = North, 180° = East, 270° = South,
    then shifted 90° clockwise (to the right).
    """
    global imu_heading_rad

    # 1) Clear and set up plot
    plot_widget.clear()
    plot_widget.setTitle("IMU Heading")
    plot_widget.showGrid(x=True, y=True)

    # 2) Handle no data
    if imu_average is None:
        txt = pg.TextItem("No IMU Data", color='r')
        txt.setPos(0, 0)
        plot_widget.addItem(txt)
        return

    # 3) Unpack averaged quaternion
    w = imu_average.get('quat_w', 1.0)
    x = imu_average.get('quat_x', 0.0)
    y = imu_average.get('quat_y', 0.0)
    z = imu_average.get('quat_z', 0.0)

    # 4) Compute raw yaw: 0°=East, 90°=North
    heading_rad = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    heading_deg = np.degrees(heading_rad)  # raw yaw

    imu_heading_rad = heading_rad

    # 5) Rotate to compass convention: 0°=West, 90°=North, etc.
    display_rad = np.pi - heading_rad
    display_deg = (180.0 - heading_deg) % 360.0

    # 6) Shift arrow 90° clockwise (to the right)
    shifted_rad = display_rad - (np.pi / 2)
    shifted_deg = (display_deg - 90.0) % 360.0

    # 7) Compute arrow tip on unit circle
    tip_x = np.cos(shifted_rad)
    tip_y = np.sin(shifted_rad)

    # 8) Draw the arrow
    plot_widget.plot([0, tip_x], [0, tip_y], pen=pg.mkPen('c', width=3))
    # 9) Label with shifted heading
    lbl = pg.TextItem(f"{shifted_deg:.1f}°", color='c', anchor=(0.5, -0.2))
    lbl.setPos(tip_x, tip_y)
    plot_widget.addItem(lbl)

    #print(f"[IMU] Heading Angle: {shifted_deg:.2f}°")

# ------------------------------
# Plot-6’s 
# ------------------------------
def plot6(plot_widget, ego_matrix):
    """
    Dedicated panel for visualizing ego-motion rotation.
    - Draws a yellow arrow showing rotation
    - Prints the 2x2 rotation matrix R^T
    - Displays the heading in degrees next to the arrow
    """
    plot_widget.clear()
    plot_widget.setTitle("Ego-Motion Rotation")
    plot_widget.showGrid(x=True, y=True)

    if ego_matrix is None:
        txt = pg.TextItem("No Ego-Motion Data", color='r')
        txt.setPos(0, 0)
        plot_widget.addItem(txt)
        return

    Rt = ego_matrix[0:2, 0:2]

    # Convert to matching IMU convention
    icp_angle = np.arctan2(Rt[1, 0], Rt[0, 0])
    display_rad = np.pi - icp_angle
    shifted_rad = display_rad - (np.pi / 2)
    shifted_deg = (np.degrees(display_rad) - 90.0) % 360.0

    # Arrow direction
    u_x = np.cos(shifted_rad)
    u_y = np.sin(shifted_rad)

    # Draw arrow
    plot_widget.plot([0, u_x], [0, u_y], pen=pg.mkPen('y', width=3))

    # Show heading text next to arrow
    lbl = pg.TextItem(f"{shifted_deg:.1f}°", color='y', anchor=(0.5, -0.2))
    lbl.setPos(u_x, u_y)
    plot_widget.addItem(lbl)

    # Print rotation matrix
    mat_text = (
        f"Rᵀ =\n"
        f"[{Rt[0,0]:.3f} {Rt[0,1]:.3f}]\n"
        f"[{Rt[1,0]:.3f} {Rt[1,1]:.3f}]"
    )
    txt = pg.TextItem(mat_text, color='y')
    txt.setPos(-1.5, -1.5)
    plot_widget.addItem(txt)

    #print(f"[ICP] Heading Angle: {shifted_deg:.2f}°")


# ------------------------------
# Main Viewer
# ------------------------------
class ClusterViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pipeline Viewer")
        self.resize(1200,900)

        # Load data once
        self.imu_frames = imuLoader.load_all(False) if imuLoader else []
        self.radarA_frames = radarLoaderA.load_all(False) if radarLoaderA else []
        self.radarB_frames = radarLoaderB.load_all(False) if radarLoaderB else []
        
        for frame in self.radarA_frames:
            for point in frame:
                point.x, point.y = helper.rotate_point_A(point.x, point.y)
                point.x += 0.70  # Adjust radar A points by +58cm on x-axis


        for frame in self.radarB_frames:
            for point in frame:
                point.x, point.y = helper.rotate_point_B(point.x, point.y)
                point.x -= 0.70  # Adjust radar A points by +58cm on x-axis

        len_a = len(self.radarA_frames)
        len_b = len(self.radarB_frames)
        max_len = max(len_a, len_b)

        # Pad shorter one
        if len_a < max_len:
            self.radarA_frames.extend([None] * (max_len - len_a))
        elif len_b < max_len:
            self.radarB_frames.extend([None] * (max_len - len_b))

        self.radarDataSetLength = max_len

        self.currentFrame = -1

        # Plot will now use spatial matching with tuned parameters; others remain default
        self.trackers = {}
        for i in range(1, 7):
            if i == 2:
                # plot2 uses our line-fit tracker
                self.trackers[f"plot{i}"] = ClusterTracker(
                    maximum_misses        = MAX_CONSECUTIVE_MISSES,
                    distance_threshold    = LINE_DISTANCE_THRESHOLD,
                    line_fit_period       = LINE_FIT_UPDATE_PERIOD,
                    maximum_history_length= NUMBER_OF_PAST_SAMPLES,
                    maximum_segment_length= MAX_HISTORY_SEGMENT_LENGTH
                )
            else:
                # other plots keep defaults
                self.trackers[f"plot{i}"] = ClusterTracker()

        # Build UI
        main_layout = QVBoxLayout(self)
        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget)
        self.plots = {}

        for idx in range(1,7):
            row, col = divmod(idx-1,3)
            p = self.plot_widget.addPlot(row=row,col=col)
            p.setXRange(-2,20); 
            p.setYRange(-2,20)
            p.setAspectLocked(True); 
            p.showGrid(x=True,y=True)
            p.setTitle(f"Plot {idx}")
            self.plots[f"plot{idx}"] = p

        p7 = self.plot_widget.addPlot(row=3, col=0, colspan=3)
        p7.setXRange(-1,1)
        p7.setYRange(-1,1)
        p7.setAspectLocked(True)
        p7.showGrid(x=True,y=True)
        p7.setTitle("Plot 7: IMU Heading")
        self.plots["plot7"] = p7

        
        # Control bar: label, slider, buttons
        ctrl = QHBoxLayout()
        self.slider_label = QLabel("Frame: 0")
        self.slider       = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.radarDataSetLength-1)
        self.slider.valueChanged.connect(self.on_slider_changed)

        self.prev_btn = QPushButton("<")
        self.next_btn = QPushButton(">")
        self.prev_btn.clicked.connect(lambda: self.change_slider(-1))
        self.next_btn.clicked.connect(lambda: self.change_slider(+1))

        ctrl.addWidget(self.slider_label)
        ctrl.addWidget(self.slider)
        ctrl.addWidget(self.prev_btn)
        ctrl.addWidget(self.next_btn)
        main_layout.addLayout(ctrl)

        main_layout.addLayout(ctrl)

        # Start at frame 0
        self.on_slider_changed(0)

    def on_slider_changed(self, newFrame):
        # rewind on backward move
        if newFrame < self.currentFrame:
            if ENABLE_SENSORS in (1, 3):
                _radarA_Agg.clearBuffer()
                _radarB_Agg.clearBuffer()
            if ENABLE_SENSORS in (2, 3):
                _imuAgg.clearBuffer()
            self.currentFrame = -1

        # aggregate only new frames
        for f in range(self.currentFrame + 1, newFrame + 1):
            if ENABLE_SENSORS in (1, 3) and f < len(self.radarA_frames):
                _radarA_Agg.updateBuffer(self.radarA_frames[f])
            if ENABLE_SENSORS in (1, 3) and f < len(self.radarB_frames):
                _radarB_Agg.updateBuffer(self.radarB_frames[f])
            
            if ENABLE_SENSORS in (2, 3) and f < len(self.imu_frames):
                _imuAgg.updateBuffer(self.imu_frames[f])

        self.currentFrame  = newFrame
        self.slider_label.setText(f"Frame: {newFrame}")
        # Call plots to be drawn
        self.update_all_plots()

    def change_slider(self, delta):
        """
        Move the frame slider by delta (±1) and clamp to [min, max].
        """
        new_val = self.slider.value() + delta
        new_val = max(self.slider.minimum(),
                      min(self.slider.maximum(), new_val))
        self.slider.setValue(new_val)


    def update_all_plots(self):
        global P, Q
        global P_global, Q_global
        global T_global
        global cumulative_distance_global, cumulative_heading_global
        global cumulative_distance_cluster, cumulative_heading_cluster
        # Filtern Pipeline information (Plot 1 only)
        if ENABLE_SENSORS in (1, 3):
            latestFrameA = _radarA_Agg.getPoints()
            latestFrameB = _radarB_Agg.getPoints()

            for point in latestFrameA:
                point["sensorId"] = "A"
            for point in latestFrameB:
                point["sensorId"]  = "B"
            rawPointCloud = latestFrameA + latestFrameB
        else:
            rawPointCloud = []
        pointCloud = pointFilter.filterDoppler(rawPointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)
        #pointCloud = pointFilter.filterSNRmin( rawPointCloud, FILTER_SNR_MIN)
        pointCloud = pointFilter.filterCartesianZ(pointCloud, FILTER_Z_MIN, FILTER_Z_MAX)
        pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)
        #pointCloud = pointFilter.filterSphericalPhi(pointCloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        
        
        """
        Apply RANSAC-based dynamic object filtering.
        Output:
        - model
        - inliers
        - outliers
        - X
        - Y
        - theta
        - doppler
        """
        ransac_output = pointFilter.filter_moving_objects_ransac(pointCloud, return_model=True)
        if ransac_output:
            filteredPointCloud = ransac_output["inliers"]
        else:
            filteredPointCloud = []
        if filteredPointCloud:
            escalarVego = selfSpeedEstimator.estimate_ego_speed_scalar(filteredPointCloud)
            vectorVego = selfSpeedEstimator.estimate_ego_velocity_vector(filteredPointCloud)

            escalarVego = Vego_filter.update(escalarVego)
        else:
            escalarVego = 0.0
            vectorVego = (0.0, 0.0)
        print(
                f"Frame {self.currentFrame}: "
                f"Velocity Scalar = {escalarVego:.4f}, "
                f"Velocity Vectorial = ({vectorVego[0]:.4f}), ({vectorVego[1]:.4f})"
            )

        imu_records = []
        if ENABLE_SENSORS in (2, 3):
            for frame in _imuAgg.frames:
                imu_records.extend(frame)
        
        imuData = helper.average_imu_records(imu_records) if imu_records else None

        # Stage 1 clustering
        clusterProcessor_stage1 = pointFilter.extract_points(filteredPointCloud)
        clusterProcessor_stage1, _ = cluster_processor_stage1.cluster_points(clusterProcessor_stage1)

        # Stage 2 clustering
        if len(clusterProcessor_stage1)>0:
            clusterProcessor_flat = np.vstack([cdata['points'] for cdata in clusterProcessor_stage1.values()])
            clusterProcessor_final, _ = cluster_processor_stage2.cluster_points(clusterProcessor_flat)
        else:
            clusterProcessor_final = {}

        # Store value that was stored in P into Q
        #Q = P
        # Obtain the current cluster set of points
        #P = clusterProcessor_final


        # now draw each plot
        for name, plot_item in self.plots.items():

            if name == "plot1" and ENABLE_SENSORS in (1, 3):
                plot1(plot_item, filteredPointCloud)
                
            if name == "plot2":
                # Plot 1: real clusters → spatial tracker 
                # update the tracker with the fresh Stage-2 clusters
                self.trackers[name].update(clusterProcessor_final)

                # pull out the tracks (persistent IDs) and their data
                clusters = self.trackers[name].get_active_tracks()

                plot2(plot_item, clusters)
            if name == "plot3":
                plot3(plot_item, rawPointCloud, ransac_output)
            
            if name == "plot4":
                plot_trajectory(plot_item, trajectory_cluster, "EgoMotion Cluster", color='g', extra_lines=[
                    {'trajectory': trajectory_cluster_imu, 'label': 'IMU-Based', 'color': 'b'}])

            if name == "plot5":
                plot_trajectory(plot_item, trajectory_global, "EgoMotion Global", color='g', extra_lines=[
                    {'trajectory': trajectory_global_imu, 'label': 'IMU-Based', 'color': 'b'}])

            if name == "plot6":
                # update the tracker with the fresh Stage-2 clusters
                self.trackers[name].update(clusterProcessor_final)

                # pull out the tracks (persistent IDs) and their data
                clusters = self.trackers[name].get_active_tracks()

                # Store value that was stored in P into Q
                Q_global = P_global # Obtained only last sample to avoid errors that could have been accumulated
                P_global = filteredPointCloud # Obtain the current cluster set of points

                resultVectors_global = icp.icp_pointCloudeWise_vectors(P_global, Q_global)

                matched_points = resultVectors_global['global']['matched_points']
                centroid_disp = resultVectors_global['global']['centroid_displacement']

                # Store value that was stored in P into Q
                Q = P # Obtained only last sample to avoid errors that could have been accumulated
                P = clusters # Obtain the current cluster set of points

                resultVectors_cluster = icp.icp_clusterWise_vectors(P, Q)

                Ticp_global = icp.icp_transformation_matrix({
                    'translation_avg': resultVectors_global['global']['translation'],
                    'rotation_avg': resultVectors_global['global']['rotation']
                })

                Ticp_cluster = icp.icp_transformation_matrix({
                    'translation_avg': resultVectors_cluster['global']['translation'],
                    'rotation_avg': resultVectors_cluster['global']['rotation']
                })

                Tego_global, Rrot_global, Rtrans_global = icp.icp_ego_motion_matrix({
                    'translation_avg': resultVectors_global['global']['translation'],
                    'rotation_avg': resultVectors_global['global']['rotation']
                })
                Tego_cluster, Rrot_cluster, Rtrans_cluster = icp.icp_ego_motion_matrix({
                    'translation_avg': resultVectors_cluster['global']['translation'],
                    'rotation_avg': resultVectors_cluster['global']['rotation']
                })

                # Extract local translation and rotation from Ticp_global
                dx_local = Tego_global[0, 2]
                dy_local = Tego_global[1, 2]
                theta_local = np.arctan2(Tego_global[1, 0], Tego_global[0, 0])

                step_distance = np.sqrt(dx_local**2 + dy_local**2)
                
                cumulative_distance_global += step_distance
                cumulative_heading_global += theta_local

                # Store in egoMotion_global
                egoMotion_global['translation'].append(cumulative_distance_global)
                egoMotion_global['rotations'].append(cumulative_heading_global)

                ####
                # Extract local translation and rotation from Ticp_global
                dx_local = Tego_cluster[0, 2]
                dy_local = Tego_cluster[1, 2]
                theta_local = np.arctan2(Tego_cluster[1, 0], Tego_cluster[0, 0])

                step_distance = np.sqrt(dx_local**2 + dy_local**2)
                
                cumulative_distance_cluster += step_distance
                cumulative_heading_cluster += theta_local

                # Store in egoMotion_global
                egoMotion_cluster['translation'].append(cumulative_distance_cluster)
                egoMotion_cluster['rotations'].append(cumulative_heading_cluster)

                # --- Update ego-motion trajectory from cluster ---
                if egoMotion_cluster['translation']:
                    last_dist = egoMotion_cluster['translation'][-1] - (egoMotion_cluster['translation'][-2] if len(egoMotion_cluster['translation']) > 1 else 0.0)
                    
                    # From ICP rotation
                    last_angle = egoMotion_cluster['rotations'][-1]
                    x_prev, y_prev = trajectory_cluster[-1]
                    x_new = x_prev + last_dist * np.sin(last_angle)
                    y_new = y_prev + last_dist * np.cos(last_angle)
                    trajectory_cluster.append((x_new, y_new))

                    # From IMU rotation (if available)
                    if imu_heading_rad is not None:
                        x_prev_imu, y_prev_imu = trajectory_cluster_imu[-1]
                        x_new_imu = x_prev_imu + last_dist * np.sin(imu_heading_rad)
                        y_new_imu = y_prev_imu + last_dist * np.cos(imu_heading_rad)
                        trajectory_cluster_imu.append((x_new_imu, y_new_imu))

                # --- Update ego-motion trajectory from global ---
                if egoMotion_global['translation']:
                    last_dist = egoMotion_global['translation'][-1] - (egoMotion_global['translation'][-2] if len(egoMotion_global['translation']) > 1 else 0.0)

                    # From ICP rotation
                    last_angle = egoMotion_global['rotations'][-1]
                    x_prev, y_prev = trajectory_global[-1]
                    x_new = x_prev + last_dist * np.sin(last_angle)
                    y_new = y_prev + last_dist * np.cos(last_angle)
                    trajectory_global.append((x_new, y_new))

                    # From IMU rotation (if available)
                    if imu_heading_rad is not None:
                        x_prev_imu, y_prev_imu = trajectory_global_imu[-1]
                        x_new_imu = x_prev_imu + last_dist * np.sin(imu_heading_rad)
                        y_new_imu = y_prev_imu + last_dist * np.cos(imu_heading_rad)
                        trajectory_global_imu.append((x_new_imu, y_new_imu))
                                
                plot6(plot_item, Tego_cluster)
            if name == "plot7" and ENABLE_SENSORS in (2, 3):
                plot7(plot_item, imuData)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
