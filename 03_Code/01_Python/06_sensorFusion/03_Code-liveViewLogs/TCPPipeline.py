import sys
import socket
import threading
import time
import numpy as np
import pyqtgraph as pg
import math
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton
)
from PyQt5.QtCore import Qt, QTimer

from ClusterTracker import ClusterTracker
import helper
from frameAggregator import FrameAggregator
import dbCluster
import pointFilter
import icp
from kalmanFilter import KalmanFilter
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

ENABLE_SENSORS = 3
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 10
FILTER_SNR_MIN                  = 12
FILTER_Z_MIN, FILTER_Z_MAX      = -2, 10
FILTER_Y_MIN, FILTER_Y_MAX      = 1.5, 15
FILTER_PHI_MIN, FILTER_PHI_MAX  = -85, 85
FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX = 0.01, 8.0

# Globals (same as before)
P = Q = None
P_global = Q_global = None
P_matched = Q_matched = None
egoMotion_global = {'translation': [], 'rotations': []}
egoMotion_cluster = {'translation': [], 'rotations': []}
cumulative_distance_global = cumulative_heading_global = 0.0
cumulative_distance_cluster = cumulative_heading_cluster = 0.0
trajectory_cluster = [(0.0, 0.0)]
trajectory_global = [(0.0, 0.0)]
trajectory_cluster_imu = [(0.0, 0.0)]
trajectory_global_imu = [(0.0, 0.0)]
imu_heading_rad = None
T_global = np.eye(3)

# Aggregators
_radarA_Agg = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_radarB_Agg = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_imuAgg     = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

# Two‐stage DBSCAN
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=3)

Vego_filter = KalmanFilter(process_variance=0.02, measurement_variance=0.5)


# ------------------------------
# Main Viewer with TCP input
# ------------------------------
class ClusterViewer(QWidget):
    def __init__(self, host="192.168.9.97", port=8888):
        super().__init__()
        self.setWindowTitle("Pipeline Viewer (TCP)")
        self.resize(1200, 900)

        # Networking
        self.host = host
        self.port = port
        self.socket = None
        self.running = True
        self.current_sensor = None
        self.radar_a_frame = []
        self.radar_b_frame = []
        self.imu_frame = []

        # Start network thread
        self.recv_thread = threading.Thread(target=self.network_loop, daemon=True)
        self.recv_thread.start()

        # Cluster trackers
        self.trackers = {}
        for i in range(1, 7):
            if i == 2:
                self.trackers[f"plot{i}"] = ClusterTracker(
                    maximum_misses        = MAX_CONSECUTIVE_MISSES,
                    distance_threshold    = LINE_DISTANCE_THRESHOLD,
                    line_fit_period       = LINE_FIT_UPDATE_PERIOD,
                    maximum_history_length= NUMBER_OF_PAST_SAMPLES,
                    maximum_segment_length= MAX_HISTORY_SEGMENT_LENGTH
                )
            else:
                self.trackers[f"plot{i}"] = ClusterTracker()

        # Build UI
        main_layout = QVBoxLayout(self)
        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget)
        self.plots = {}

        for idx in range(1,7):
            row, col = divmod(idx-1,3)
            p = self.plot_widget.addPlot(row=row,col=col)
            p.setXRange(-2,20); p.setYRange(-2,20)
            p.setAspectLocked(True); p.showGrid(x=True,y=True)
            p.setTitle(f"Plot {idx}")
            self.plots[f"plot{idx}"] = p

        p7 = self.plot_widget.addPlot(row=3, col=0, colspan=3)
        p7.setXRange(-1,1); p7.setYRange(-1,1)
        p7.setAspectLocked(True); p7.showGrid(x=True,y=True)
        p7.setTitle("Plot 7: IMU Heading")
        self.plots["plot7"] = p7

        # Timer for periodic updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all_plots)
        self.timer.start(100)

    # ------------------------------
    # TCP Networking
    # ------------------------------
    def network_loop(self):
        while self.running:
            try:
                print(f"[INFO] Connecting to {self.host}:{self.port}...")
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                print("[INFO] Connected!")

                buffer = ""
                while self.running:
                    data = self.socket.recv(4096)
                    if not data:
                        raise ConnectionError("Server disconnected")
                    buffer += data.decode("utf-8")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        self.parse_line(line.strip())

            except Exception as e:
                print(f"[WARN] Connection lost: {e}")
                if self.socket:
                    self.socket.close()
                    self.socket = None
                time.sleep(2)

    def parse_line(self, line):
        if not line:
            return
        parts = line.split(",")

        # ---------------- RADAR A ----------------
        if parts[0] == "RADAR_A":
            if self.radar_a_frame:
                _radarA_Agg.updateBuffer(self.radar_a_frame)
            self.radar_a_frame = []
            self.current_sensor = "A"
            print("[RADAR_A] New frame header received")

        # ---------------- RADAR B ----------------
        elif parts[0] == "RADAR_B":
            if self.radar_b_frame:
                _radarB_Agg.updateBuffer(self.radar_b_frame)
            self.radar_b_frame = []
            self.current_sensor = "B"
            print("[RADAR_B] New frame header received")

        # ---------------- IMU ----------------
        elif parts[0] == "IMU" and len(parts) == 5:
            try:
                qw, qx, qy, qz = map(float, parts[1:])
                imu_point = type("IMU", (), {
                    "quat_w": qw, "quat_x": qx,
                    "quat_y": qy, "quat_z": qz
                })()
                _imuAgg.updateBuffer([imu_point])

                # ✅ Debug print
                print(f"[IMU parsed] w={imu_point.quat_w:.3f}, "
                    f"x={imu_point.quat_x:.3f}, "
                    f"y={imu_point.quat_y:.3f}, "
                    f"z={imu_point.quat_z:.3f}")

            except Exception as e:
                print(f"[WARN] IMU parse error: {e}, line={line}")

        # ---------------- Radar point data ----------------
        elif parts[0].isdigit():
            try:
                _, x, y, z, doppler = parts
                x, y, z = float(x), float(y), float(z)
                doppler = float(doppler)

                if self.current_sensor == "A":
                    x, y = helper.rotate_point_A(x, y)
                    x -= 0.32
                    x, y, z = helper.compensate_pitch(x, y, z)
                    self.radar_a_frame.append(
                        type("P", (), {
                            "x": x, "y": y, "z": z,
                            "doppler": doppler, "snr": 0, "noise": 0
                        })()
                    )

                elif self.current_sensor == "B":
                    x, y = helper.rotate_point_B(x, y)
                    x -= 0.28
                    x, y, z = helper.compensate_pitch(x, y, z)
                    self.radar_b_frame.append(
                        type("P", (), {
                            "x": x, "y": y, "z": z,
                            "doppler": doppler, "snr": 0, "noise": 0
                        })()
                    )

            except Exception as e:
                print(f"[WARN] Radar point parse error: {e}, line={line}")


    # ------------------------------
    # Update all plots (unchanged pipeline logic)
    # ------------------------------
    def update_all_plots(self):
        global T_global, P_global

        # ---------------- Radar Aggregation ----------------
        latestFrameA = _radarA_Agg.getPoints()
        latestFrameB = _radarB_Agg.getPoints()
        rawPointCloud = latestFrameA + latestFrameB

        # ✅ Debug print
        print(f"[Radar agg] A={len(latestFrameA)} points, "
            f"B={len(latestFrameB)} points, "
            f"Total={len(rawPointCloud)}")
        if rawPointCloud:
            for i, pt in enumerate(rawPointCloud[:3]):
                print(f"   Radar[{i}] x={pt['x']:.3f}, y={pt['y']:.3f}, "
                    f"z={pt['z']:.3f}, doppler={pt['doppler']:.3f}")
            if len(rawPointCloud) > 3:
                print("   ...")

        # ---------------- Filtering ----------------
        pointCloud = pointFilter.filterDoppler(rawPointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)
        pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)

        # ---------------- IMU Aggregation ----------------
        imu_records = []
        for frame in _imuAgg.frames:
            imu_records.extend(frame)

        if imu_records:
            latest_imu = imu_records[-1]
            print(f"[IMU agg] {len(imu_records)} records, "
                f"latest: w={latest_imu.quat_w:.3f}, "
                f"x={latest_imu.quat_x:.3f}, "
                f"y={latest_imu.quat_y:.3f}, "
                f"z={latest_imu.quat_z:.3f}")
            imuData = helper.average_imu_records(imu_records)
            print(f"[IMU avg] {imuData}")
        else:
            print("[IMU agg] no records yet")
            imuData = None

        # ---------------- RANSAC Filtering ----------------
        ransac_output = pointFilter.filter_moving_objects_ransac(pointCloud, return_model=True)
        filteredPointCloud = ransac_output["inliers"] if ransac_output else []

        # ---------------- ICP Update ----------------
        P = np.array([[p['x'], p['y']] for p in filteredPointCloud]) if filteredPointCloud else None
        if P is not None and P_global is not None and len(P) > 0:
            try:
                T_global, _ = icp.run_icp(P, P_global, T_global)
                print(f"[ICP] Updated T_global:\n{T_global}")
            except Exception as e:
                print(f"[ICP] Failed: {e}")
        P_global = P

        # ---------------- Plot Updates ----------------
        for name, plot_item in self.plots.items():
            if name == "plot1":
                plot1(plot_item, filteredPointCloud)
            elif name == "plot2":
                self.trackers[name].update({})
                plot2(plot_item, {})
            elif name == "plot3":
                plot3(plot_item, rawPointCloud, ransac_output)
            elif name == "plot7":
                plot7(plot_item, imuData)
            elif name == "plot6":
                plot6(plot_item, T_global)



# ------------------------------
# Plot-1’s custom view (unchanged)
# ------------------------------
def plot1(plot_widget, pointCloud):
    """
    Render only the filtered point cloud (pointCloud),
    plotting Sensor A and Sensor B in different colors.
    """

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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
