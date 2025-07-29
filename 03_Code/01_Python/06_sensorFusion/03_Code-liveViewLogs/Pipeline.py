import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton
)
from PyQt5.QtCore import Qt, QTimer
import socket
import threading
import queue
import math

from ClusterTracker import ClusterTracker
import helper
from frameAggregator import FrameAggregator
import dbCluster
import pointFilter
import icp

# -------------------------------------------------------------
# PARAMETERS
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 10
FILTER_SNR_MIN                  = 12
FILTER_Z_MIN, FILTER_Z_MAX      = -2, 2
FILTER_Y_MIN, FILTER_Y_MAX      = 0.6, 15
FILTER_PHI_MIN, FILTER_PHI_MAX  = -85, 85
FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX = 0.01, 8.0
TRACKER_MAX_MISSES  = 10
TRACKER_DIST_THRESH = 0.5

TCP_IP = "192.168.63.97"
TCP_PORT = 9000

# -------------------------------------------------------------
# Globals
# -------------------------------------------------------------
P = None
Q = None
icp_history = {
    'result_vectors':    [],
    'motion_vectors':    [],
    'world_transforms':  [],
    'ego_transforms':    []
}

# Aggregators
_radarAgg = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_imuAgg   = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

# DBSCAN processors
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=5)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=2)

# TCP Queue
data_queue = queue.Queue()

# -------------------------------------------------------------
# TCP Reader (blocking until connected)
# -------------------------------------------------------------
def tcp_reader_blocking():
    print("[INFO] Waiting for TCP connection...")
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False
    while not connected:
        try:
            tcp.connect((TCP_IP, TCP_PORT))
            connected = True
        except (ConnectionRefusedError, TimeoutError):
            print("[INFO] Connection failed, retrying...")
            import time; time.sleep(1)

    tcp_file = tcp.makefile('r')
    print("[INFO] TCP Connected!")
    while True:
        line = tcp_file.readline()
        if not line:
            break
        data_queue.put(line.strip())

# -------------------------------------------------------------
# Parsers
# -------------------------------------------------------------
def parse_radar(line):
    tokens = line.replace('[RADAR] ', '').split()
    return {
        'x': float(tokens[2].split('=')[1]),
        'y': float(tokens[3].split('=')[1]),
        'z': float(tokens[4].split('=')[1]),
        'doppler': float(tokens[5].split('=')[1]),
        'snr': int(tokens[6].split('=')[1])
    }

def parse_imu(line):
    tokens = line.replace('[IMU] ', '').split()
    q = [float(x) for x in tokens[2].split('=')[1].strip('()').split(',')]
    return {'quat': q}

def compute_yaw(imu):
    qw, qx, qy, qz = imu['quat']
    return math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))

# -------------------------------------------------------------
# Plotting Functions (same as original projectQT)
# -------------------------------------------------------------
def plot1(plot_widget, clusters, predictions):
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud")
    for cid,data in clusters.items():
        clusterPoints = data['points']
        clusterDoppler = data['doppler_avg']
        clusterHits    = data.get('hits', 0)
        if clusterPoints.shape[0] > 0:
            scatter = pg.ScatterPlotItem(
                x=clusterPoints[:,0], y=clusterPoints[:,1], size=8,
                pen=None, brush=pg.mkBrush(0,200,0,150)
            )
            plot_widget.addItem(scatter)
        cx, cy = data['centroid'][:2]
        label = pg.TextItem(
            f"ID: {cid}\nDoppler: {clusterDoppler:.2f}\nHits: {clusterHits}\nCx,Cy: ({cx:.2f}, {cy:.2f})",
            anchor=(0.5, -0.2),
            color='w'
        )
        label.setPos(cx, cy)
        plot_widget.addItem(label)
    for cid,(px,py,*_) in predictions.items():
        pred_sc = pg.ScatterPlotItem(x=[px], y=[py], symbol='x', size=14, pen=pg.mkPen('r', width=2))
        plot_widget.addItem(pred_sc)
        lbl = pg.TextItem(f"Pred {cid}", color='r')
        lbl.setPos(px+0.3, py+0.3)
        plot_widget.addItem(lbl)

def plot2(plot_widget, clusters, predictions):
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud")
    for cid,data in clusters.items():
        clusterPoints = data['points']
        clusterDoppler = data['doppler_avg']
        clusterHits    = data.get('hits', 0)
        if clusterPoints.shape[0] > 0:
            scatter = pg.ScatterPlotItem(
                x=clusterPoints[:,0], y=clusterPoints[:,1], size=8,
                pen=None, brush=pg.mkBrush(0,200,0,150)
            )
            plot_widget.addItem(scatter)
        cx, cy = data['centroid'][:2]
        label = pg.TextItem(
            f"ID: {cid}\nDoppler: {clusterDoppler:.2f}\nHits: {clusterHits}\nCx,Cy: ({cx:.2f}, {cy:.2f})",
            anchor=(0.5, -0.2),
            color='w'
        )
        label.setPos(cx, cy)
        plot_widget.addItem(label)
    for cid,(px,py,*_) in predictions.items():
        pred_sc = pg.ScatterPlotItem(x=[px], y=[py], symbol='x', size=14, pen=pg.mkPen('r', width=2))
        plot_widget.addItem(pred_sc)
        lbl = pg.TextItem(f"Pred {cid}", color='r')
        lbl.setPos(px+0.3, py+0.3)
        plot_widget.addItem(lbl)

def plot3(plot_widget, ego_matrix):
    plot_widget.clear()
    plot_widget.setTitle("Ego-Motion Rotation")
    plot_widget.enableAutoRange(False)
    plot_widget.setXRange(-2, 2)
    plot_widget.setYRange(-2, 2)
    plot_widget.setAspectLocked(True)
    plot_widget.showGrid(x=True, y=True)

    if ego_matrix is None:
        txt = pg.TextItem("No Ego-Motion Data", color='r')
        txt.setPos(0, 0)
        plot_widget.addItem(txt)
        return

    Rt = ego_matrix[0:2, 0:2]
    v = np.array([1.0, 0.0])
    u = Rt.dot(v)
    plot_widget.plot([0, u[0]], [0, u[1]], pen=pg.mkPen('y', width=3))
    mat_text = f"Rᵀ =\n[{Rt[0,0]:.3f} {Rt[0,1]:.3f}]\n[{Rt[1,0]:.3f} {Rt[1,1]:.3f}]"
    txt = pg.TextItem(mat_text, color='y')
    txt.setPos(-1.5, -1.5)
    plot_widget.addItem(txt)

def plot4(plot_widget, ego_matrix, translation_history):
    plot_widget.clear()
    plot_widget.setTitle("Ego-Motion Translation")
    plot_widget.enableAutoRange(False)
    plot_widget.setXRange(-10, 10)
    plot_widget.setYRange(-10, 10)
    plot_widget.setAspectLocked(True)
    plot_widget.showGrid(x=True, y=True)

    if ego_matrix is None:
        txt = pg.TextItem("No Ego-Motion Data", color='r')
        txt.setPos(0, 0)
        plot_widget.addItem(txt)
        return

    tx = ego_matrix[0, 2]
    ty = ego_matrix[1, 2]
    translation_history.append((tx, ty))

    if len(translation_history) > 0:
        xs, ys = zip(*translation_history)
        scatter = pg.ScatterPlotItem(x=xs, y=ys, symbol='x', size=10, pen=pg.mkPen('b', width=2))
        plot_widget.addItem(scatter)

    txt = pg.TextItem(f"Translation:\nX={tx:.2f}, Y={ty:.2f}", color='b')
    txt.setPos(tx, ty)
    plot_widget.addItem(txt)

# -------------------------------------------------------------
# Main Viewer
# -------------------------------------------------------------
class ClusterViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel Cluster Viewer")
        self.resize(1200, 900)

        self.radar_points_buffer = []
        self.imu_buffer = []
        self.currentFrame = 0

        self.trackers = {
            f"plot{i}": (
                ClusterTracker(max_misses=TRACKER_MAX_MISSES, dist_threshold=TRACKER_DIST_THRESH)
                if i == 1 else ClusterTracker()
            )
            for i in range(1, 5)
        }

        main_layout = QVBoxLayout(self)
        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget)
        self.plots = {}
        for idx in range(1, 5):
            row, col = divmod(idx-1, 3)
            p = self.plot_widget.addPlot(row=row, col=col)
            p.setXRange(-2, 20)
            p.setYRange(-2, 20)
            p.setAspectLocked(True)
            p.showGrid(x=True, y=True)
            p.setTitle(f"Plot {idx}")
            self.plots[f"plot{idx}"] = p

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all_plots)
        self.timer.start(50)

    def poll_tcp_data(self):
        while not data_queue.empty():
            line = data_queue.get()
            if line.startswith("[RADAR]"):
                self.radar_points_buffer.append(parse_radar(line))
            elif line.startswith("[IMU]"):
                self.imu_buffer.append(parse_imu(line))

        if len(self.radar_points_buffer) > 0:
            _radarAgg.updateBuffer(self.radar_points_buffer)
            self.radar_points_buffer.clear()

        if len(self.imu_buffer) > 0:
            _imuAgg.updateBuffer(self.imu_buffer)
            self.imu_buffer.clear()

    def update_all_plots(self):
        global P, Q, icp_history
        self.poll_tcp_data()

        rawPointCloud = _radarAgg.getPoints()
        pointCloud = pointFilter.filterSNRmin(rawPointCloud, FILTER_SNR_MIN)
        pointCloud = pointFilter.filterCartesianZ(pointCloud, FILTER_Z_MIN, FILTER_Z_MAX)
        pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)
        pointCloud = pointFilter.filterSphericalPhi(pointCloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filteredPointCloud = pointFilter.filterDoppler(pointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

        clusterProcessor_stage1 = pointFilter.extract_points(filteredPointCloud)
        clusterProcessor_stage1, _ = cluster_processor_stage1.cluster_points(clusterProcessor_stage1)

        if len(clusterProcessor_stage1) > 0:
            clusterProcessor_flat = np.vstack([cdata['points'] for cdata in clusterProcessor_stage1.values()])
            clusterProcessor_final, _ = cluster_processor_stage2.cluster_points(clusterProcessor_flat)
        else:
            clusterProcessor_final = {}

        for name, plot_item in self.plots.items():
            self.trackers[name].update(clusterProcessor_final)
            clusters = self.trackers[name].get_active_tracks()
            preds = self.trackers[name].get_predictions()

            if name == "plot1":
                plot1(plot_item, clusters, preds)
            elif name == "plot2":
                plot2(plot_item, clusters, preds)
            elif name == "plot3":
                Q = P
                P = clusters
                resultVectors = icp.icp_translation_vector(P, Q)
                icp_history['result_vectors'].append(resultVectors)
                motionVectors = icp.icp_get_transformation_average(resultVectors)
                icp_history['motion_vectors'].append(motionVectors)
                worldMotion = icp.icp_transformation_matrix(motionVectors)
                icp_history['world_transforms'].append(worldMotion)
                Tego = icp.icp_ego_motion_matrix(motionVectors)
                icp_history['ego_transforms'].append(Tego)
                plot3(plot_item, Tego)
            elif name == "plot4":
                Q = P
                P = clusters
                resultVectors = icp.icp_translation_vector(P, Q)
                icp_history['result_vectors'].append(resultVectors)
                motionVectors = icp.icp_get_transformation_average(resultVectors)
                icp_history['motion_vectors'].append(motionVectors)
                worldMotion = icp.icp_transformation_matrix(motionVectors)
                icp_history['world_transforms'].append(worldMotion)
                Tego = icp.icp_ego_motion_matrix(motionVectors)
                icp_history['ego_transforms'].append(Tego)

                if not hasattr(self, "translation_history"):
                    self.translation_history = []
                plot4(plot_item, Tego, self.translation_history)

# -------------------------------------------------------------
# Main Entry
# -------------------------------------------------------------
if __name__ == "__main__":
    # Start blocking TCP thread first
    tcp_thread = threading.Thread(target=tcp_reader_blocking, daemon=True)
    tcp_thread.start()

    # Wait until connection established (queue starts filling)
    print("[INFO] Blocking until TCP connection is active...")
    while data_queue.empty():
        import time; time.sleep(0.5)

    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
