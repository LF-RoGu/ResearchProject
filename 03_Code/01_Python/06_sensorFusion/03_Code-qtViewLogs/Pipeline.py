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
FILTER_Z_MIN, FILTER_Z_MAX      = -2, 2
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
egoMotion = {
    'result_vectors':    [],  # Store raw ICP results for debugging
    'T_global':          [],  # Store cumulative transformation matrix
    'translation':       [],  # Store cumulative global translations (x, y)
    'rotations':         []   # Store cumulative global heading (theta)
}
pointCloudTransform = {     # Global dictionary for transformed point clouds
    "Left": [],
    "Right": [],
    "Vehicle": []
}

cumulativeTego = np.eye(3)  # 3x3 identity (no translation/rotation yet)
T_global = np.eye(3)  # initial pose at origin
positions = []        # store global translation only
rotations = []        # store global rotation only

def yaw_rotation_matrix(yaw_deg):
    yaw_rad = np.deg2rad(yaw_deg)
    cos_y = np.cos(yaw_rad)
    sin_y = np.sin(yaw_rad)
    return np.array([
        [cos_y, -sin_y, 0],
        [sin_y,  cos_y, 0],
        [0,      0,     1]
    ])

TRANSFORMATIONS = {
    "left": {
        "R": yaw_rotation_matrix(30),        # pointing 29° from Y+ toward -X
        "T": np.array([-0.29, 0.0, 0.15])
    },
    "right": {
        "R": yaw_rotation_matrix(-30),        # pointing 29° from Y+ toward +X
        "T": np.array([0.29, 0.0, 0.15])
    }
}


folderName = "12_calibTesting"  # Folder where CSV files are stored
testType = "calibStraightWallSameConfig3.csv"  # Type of test data
# Instantiate readers and global aggregators
radarRightLoader = RadarCSVReader("radarRight_" + testType, folderName) if ENABLE_SENSORS in (1, 3) else None
radarLeftLoader = RadarCSVReader("radarLeft_" + testType, folderName) if ENABLE_SENSORS in (1, 3) else None
imuLoader   = ImuCSVReader("imu_" + testType, folderName) if ENABLE_SENSORS in (2, 3) else None
_radarAgg         = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_imuAgg           = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

# Two‐stage DBSCAN processors
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.5, min_samples=3)

Vego_filter = KalmanFilter(process_variance=0.02, measurement_variance=0.5)

def transform_pointcloud(points, sensor_label):
    """
    Apply R * point + T for each point in the list.
    Accepts either RadarRecord objects or dicts with keys x, y, z.
    """
    R = TRANSFORMATIONS[sensor_label]["R"]
    T = TRANSFORMATIONS[sensor_label]["T"]
    result = []

    for p in points:
        # RadarRecord object
        if hasattr(p, 'x') and hasattr(p, 'y') and hasattr(p, 'z'):
            local = np.array([p.x, p.y, p.z], dtype=float)
            global_coords = np.dot(R, local) + T

            result.append({
                'x': global_coords[0],
                'y': global_coords[1],
                'z': global_coords[2],
                'doppler': getattr(p, 'doppler', 0.0),
                'snr': getattr(p, 'snr', 0.0),
                'noise': getattr(p, 'noise', 0.0),
                'frame_id': getattr(p, 'frame_id', -1),
                'point_id': getattr(p, 'point_id', -1),
                'source': sensor_label
            })

        # Dictionary point
        elif isinstance(p, dict):
            if not all(k in p for k in ('x', 'y', 'z')):
                continue
            local = np.array([p['x'], p['y'], p['z']], dtype=float)
            global_coords = np.dot(R, local) + T

            p_new = p.copy()
            p_new['x'], p_new['y'], p_new['z'] = global_coords
            p_new['source'] = sensor_label
            result.append(p_new)

    return result


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
    Render the filtered point cloud (pointCloud),
    coloring by 'source' field: 'right' vs 'left'.
    """

    # 1) Clear previous items and set title
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud - Filtered")

    # 2) Bail out if empty
    if not pointCloud:
        return

    # 3) Separate points by source
    x_right, y_right = [], []
    x_left, y_left = [], []

    for p in pointCloud:
        if isinstance(p, dict) and 'x' in p and 'y' in p and 'source' in p:
            if p['source'] == 'right':
                x_right.append(p['x'])
                y_right.append(p['y'])
            elif p['source'] == 'left':
                x_left.append(p['x'])
                y_left.append(p['y'])

    # 4) Plot each group in different color
    if x_right:
        scatter_right = pg.ScatterPlotItem(
            x=x_right, y=y_right,
            size=8,
            pen=None,
            brush=pg.mkBrush(0, 200, 0, 150)  # Green for right
        )
        plot_widget.addItem(scatter_right)

    if x_left:
        scatter_left = pg.ScatterPlotItem(
            x=x_left, y=y_left,
            size=8,
            pen=None,
            brush=pg.mkBrush(0, 0, 255, 150)  # Blue for left
        )
        plot_widget.addItem(scatter_left)

    # 5) Add doppler as yellow labels for all
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
# Plot-5’s 
# ------------------------------
def plot5(plot_widget, imu_average):
    """
    Display IMU heading on a unit circle, rotated so that
    0° = West, 90° = North, 180° = East, 270° = South,
    then shifted 90° clockwise (to the right).
    """

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
        self.imu_frames = imuLoader.load_all() if imuLoader else []
        # Set both frames
        self.radarRight_frames = radarRightLoader.load_all() if radarRightLoader else []
        self.radarLeft_frames = radarLeftLoader.load_all() if radarLeftLoader else []
        if len(self.radarRight_frames) > len(self.radarLeft_frames):
            self.radarDataSetLength = len(self.radarRight_frames)
        elif len(self.radarLeft_frames) > len(self.radarRight_frames):
            self.radarDataSetLength = len(self.radarLeft_frames)
        elif len(self.radarLeft_frames) == len(self.radarRight_frames):
            self.radarDataSetLength = int((len(self.radarLeft_frames) + len(self.radarRight_frames)) / 2)
        else:
            self.radarDataSetLength = 0

        self.currentFrame = -1

        self.i_right = 0
        self.i_left = 0
        self.i_imu = 0


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

        p5 = self.plot_widget.addPlot(row=3, col=0, colspan=3)
        p5.setXRange(-1,1); p5.setYRange(-1,1)
        p5.setAspectLocked(True); p5.showGrid(x=True,y=True)
        p5.setTitle("Plot 5: IMU Heading")
        self.plots["plot5"] = p5

        
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
        global pointCloudTransform
        # rewind on backward move
        if newFrame < self.currentFrame:
            if ENABLE_SENSORS in (1, 3):
                _radarAgg.clearBuffer()
            if ENABLE_SENSORS in (2, 3):
                _imuAgg.clearBuffer()
            self.i_right = 0
            self.i_left = 0
            self.i_imu = 0
            self.currentFrame = -1

        # aggregate only new frames
        for f in range(self.currentFrame + 1, newFrame + 1):
            if ENABLE_SENSORS in (1, 3) and f < len(self.radarRight_frames):
                dataRight = self.radarRight_frames[self.i_right]
                dataLeft = self.radarLeft_frames[self.i_left]

                rightFrameID = dataRight[0].frame_id if dataRight else -1
                leftFrameID  = dataLeft[0].frame_id if dataLeft else -2

                pointCloudTransform = {
                    "Right": [],
                    "Left": [],
                    "Vehicle": []
                }

                if rightFrameID == leftFrameID:
                    pointCloudTransform["Right"] = transform_pointcloud(self.radarRight_frames[self.i_right], "right")
                    pointCloudTransform["Left"] = transform_pointcloud(self.radarLeft_frames[self.i_left], "left")
                    self.i_right += 1
                    self.i_left += 1
                elif rightFrameID < leftFrameID:
                    pointCloudTransform["Right"] = transform_pointcloud(self.radarRight_frames[self.i_right], "right")
                    self.i_right += 1
                else:
                    pointCloudTransform["Left"] = transform_pointcloud(self.radarLeft_frames[self.i_left], "left")
                    self.i_left += 1

                pointCloudTransform["Vehicle"] = pointCloudTransform["Right"] + pointCloudTransform["Left"]

                #currentFramePointCloud = self.radarRight_frames[f]
                currentFramePointCloud = self.radarLeft_frames[f] + self.radarRight_frames[f]
                _radarAgg.updateBuffer(currentFramePointCloud)
            if ENABLE_SENSORS in (2, 3) and f < len(self.imu_frames):
                if rightFrameID == leftFrameID:
                    self.i_imu += 1
                elif rightFrameID > leftFrameID:
                    self.i_imu
                else:
                    self.i_imu
                _imuAgg.updateBuffer(self.imu_frames[self.i_imu])

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
        global T_global, positions, rotations
        # Filtern Pipeline information (Plot 1 only)
        rawPointCloud = pointCloudTransform["Vehicle"] if ENABLE_SENSORS in (1, 3) else []
        pointCloud = pointFilter.filterDoppler(rawPointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)
        #pointCloud = pointFilter.filterSNRmin( rawPointCloud, FILTER_SNR_MIN)
        #pointCloud = pointFilter.filterCartesianZ(pointCloud, FILTER_Z_MIN, FILTER_Z_MAX)
        #pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)
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
            if name == "plot5" and ENABLE_SENSORS in (2, 3):
                plot5(plot_item, imuData)
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

                """
                If everything is correct, both checks should print True, meaning your ego-motion is a valid inverse of the ICP transformation.
                """
                identity_global = np.dot(Tego_global, Ticp_global)
                #print(f"Validation Global: \n {identity_global}")
                identity_cluster = np.dot(Tego_cluster, Ticp_cluster)
                #print(f"Validation Cluster: \n {identity_cluster}")

                #print("Global close to Identity:", np.allclose(identity_global, np.eye(3), atol=1e-6)) # Checks if every element is close to identity within tolerance 1e-6
                #print("Cluster close to Identity:", np.allclose(identity_cluster, np.eye(3), atol=1e-6)) # Checks if every element is close to identity within tolerance 1e-6

                # Extract local translation and rotation from Ticp_global
                tx_local = Tego_global[0, 2]
                ty_local = Tego_global[1, 2]
                theta_local = np.arctan2(Tego_global[1, 0], Tego_global[0, 0])

                # Convert local translation to world frame
                theta_global = np.arctan2(T_global[1, 0], T_global[0, 0])
                cos_g, sin_g = np.cos(theta_global), np.sin(theta_global)
                tx_world = cos_g * tx_local - sin_g * ty_local
                ty_world = sin_g * tx_local + cos_g * ty_local

                # Update T_global manually
                T_global[0, 2] += tx_world
                T_global[1, 2] += ty_world
                # Update heading
                new_theta = theta_global + theta_local
                rotations.append(new_theta)
                positions.append((T_global[0, 2], T_global[1, 2]))

                """
                print(
                    f"Frame {self.currentFrame}: "
                    f"Matched Points = {matched_points}, "
                    f"Centroid Δ = ({centroid_disp[0]:.4f}, {centroid_disp[1]:.4f}), "
                    f"ICP Tx = {tx_local:.4f}, ICP Ty = {ty_local:.4f}"
                )
                """

                """               
                print(
                    f"Frame {self.currentFrame}: "
                    f"Local Tx = {tx_local:.4f}, Local Ty = {ty_local:.4f}, "
                    f"Local Theta = {np.degrees(theta_local):.2f}°, "
                    f"World Tx = {tx_world:.4f}, World Ty = {ty_world:.4f}, "
                    f"Cumulative Pose = ({T_global[0, 2]:.4f}, {T_global[1, 2]:.4f}), "
                    f"Cumulative Heading = {np.degrees(new_theta):.2f}°"
                )
                """
                plot6(plot_item, Tego_cluster)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
