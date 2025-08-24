import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer
import socket
import threading
import queue
import time

from ClusterTracker import ClusterTracker
import helper
from frameAggregator import FrameAggregator
import dbCluster
import pointFilter
import icp

from collections import defaultdict
from odoLog import logging

# ---------------- PARAMETERS ----------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 10
FILTER_SNR_MIN = 12
FILTER_Z_MIN, FILTER_Z_MAX = -2, 2
FILTER_Y_MIN, FILTER_Y_MAX = 0.6, 15
FILTER_PHI_MIN, FILTER_PHI_MAX = -85, 85
FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX = 0.00, 8.0
TRACKER_MAX_MISSES = 10
TRACKER_DIST_THRESH = 0.5

TCP_IP = "192.168.135.97"
TCP_PORT = 9000

# ---------------- Flags ----------------
# 0 = logs, 1 = live TCP
VISUALIZATION_MODE = 1

# 1 = radar only, 2 = IMU only, 3 = both
ENABLE_SENSORS = 1

# Global variables for current (P) and previous (Q) frame clusters
P = None  # clusters for current frame t
Q = None  # clusters for previous frame t-1
icp_history = {
    'result_vectors':   [], 
    'motion_vectors':   [],
    'world_transforms': [], 
    'ego_transforms':   []
}

cumulativeTego = np.eye(3)  # 3x3 identity (no translation/rotation yet)

_radarAgg = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_imuAgg = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=5)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=2)

data_queue = queue.Queue()

filteredPointCloud_global = None

class RadarPoint:
    def __init__(self, x, y, z, doppler, snr, noise):
        self.x = x
        self.y = y
        self.z = z
        self.doppler = doppler
        self.snr = snr
        self.noise = noise

# ---------------- TCP Reader ----------------
def tcp_reader_nonblocking():
    logging.info("[INFO] Waiting for TCP connection...")
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False
    while not connected:
        try:
            tcp.connect((TCP_IP, TCP_PORT))
            connected = True
        except (ConnectionRefusedError, TimeoutError):
            logging.info("[INFO] Connection failed, retrying...")
            time.sleep(1)

    tcp.setblocking(False)
    buffer = ""
    logging.info("[INFO] TCP Connected!")

    while True:
        try:
            data = tcp.recv(4096).decode('utf-8')
            if not data:
                break
            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                data_queue.put(line.strip())
        except BlockingIOError:
            time.sleep(0.01)

# ---------------- Parsers ----------------
def parse_radar(line):
    tokens = line.replace('[RADAR] ', '').split()
    return {
        'frame': int(tokens[0].split('=')[1]),
        'pointId': int(tokens[1].split('=')[1]),
        'x': float(tokens[2].split('=')[1]),
        'y': float(tokens[3].split('=')[1]),
        'z': float(tokens[4].split('=')[1]),
        'doppler': float(tokens[5].split('=')[1]),
        'snr': int(tokens[6].split('=')[1]),
        'noise': float(tokens[7].split('=')[1])
    }

def parse_imu(line):
    tokens = line.replace('[IMU] ', '').split()
    quat = [float(x) for x in tokens[2].split('=')[1].strip('()').split(',')]
    accel = [float(x) for x in tokens[3].split('=')[1].strip('()').split(',')]
    return {
        'frame': int(tokens[0].split('=')[1]),
        'idx': int(tokens[1].split('=')[1]),
        'quat': quat,
        'accel': accel,
        'packet': int(tokens[4].split('=')[1])
    }

# ---------------- Viewer ----------------
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
            ) for i in range(1, 5)
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
            if line.startswith("[RADAR]") and (ENABLE_SENSORS in [1, 3]):
                self.radar_points_buffer.append(parse_radar(line))
            elif line.startswith("[IMU]") and (ENABLE_SENSORS in [2, 3]):
                self.imu_buffer.append(parse_imu(line))

        if self.radar_points_buffer and (ENABLE_SENSORS in [1, 3]):
            frame_groups = defaultdict(list)
            for p in self.radar_points_buffer:
                frame_groups[p['frame']].append(
                    RadarPoint(p['x'], p['y'], p['z'], p['doppler'], p['snr'], p['noise'])
                )
            for frame_id, radar_points in frame_groups.items():
                _radarAgg.updateBuffer(radar_points)
            self.radar_points_buffer.clear()

        if self.imu_buffer and (ENABLE_SENSORS in [2, 3]):
            _imuAgg.updateBuffer(self.imu_buffer)
            self.imu_buffer.clear()

    def update_all_plots(self):
        
        global P, Q, icp_history
        global filteredPointCloud_global
        global cumulativeTego
        if VISUALIZATION_MODE == 1:
            self.poll_tcp_data()
        else:
            # Future: handle reading from logs here
            pass

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

            Q = P
            P = clusters

            if name == "plot1":
                plot1(plot_item, clusters, preds)
            elif name == "plot2":
                plot2(plot_item, clusters, preds)
            elif name == "plot3":
                
                resultVectors = icp.icp_translation_vector(P, Q)
                icp_history['result_vectors'].append(resultVectors)
                motionVectors = icp.icp_get_transformation_average(resultVectors)
                icp_history['motion_vectors'].append(motionVectors)
                Tcip = icp.icp_transformation_matrix(motionVectors)
                icp_history['world_transforms'].append(Tcip)
                Tego, _, _ = icp.icp_ego_motion_matrix(motionVectors)
                icp_history['ego_transforms'].append(Tego)

                if Tego is not None:
                    cumulativeTego = np.dot(cumulativeTego, Tego)

                    logging.info(f"\n--- Frame Update #{len(icp_history['ego_transforms'])} ---")
                    logging.info(f"[ICP] Ticp (Environment Transform):\n{Tcip}")
                    logging.info(f"[EGO] Tego (Ego-Motion Transform):\n{Tego}")
                    logging.info(f"[EGO] Cumulative Ego Transform:\n{cumulativeTego}")

                    dx_step = Tego[0, 2]
                    dy_step = Tego[1, 2]
                    step_distance = np.sqrt(dx_step**2 + dy_step**2)

                    if cumulativeTego is not None:
                        dx_cumulative = cumulativeTego[0, 2]
                        dy_cumulative = cumulativeTego[1, 2]
                        cumulative_distance = np.sqrt(dx_cumulative**2 + dy_cumulative**2)

                    # Optional: compare cumulative to product of all history matrices
                    if len(icp_history['ego_transforms']) > 1:
                        history_product = np.eye(3)
                        for mat in icp_history['ego_transforms']:
                            history_product = np.dot(history_product, mat)
                        
                        logging.info(f"History Product (Validation): {history_product}")

                logging.info(
                    f"[Distance] Step={step_distance:.4f} m | "
                    f"Cumulative={cumulative_distance:.4f} m"
                )

                plot3(plot_item, Tego)
            elif name == "plot4":

                resultVectors = icp.icp_translation_vector(P, Q)
                motionVectors = icp.icp_get_transformation_average(resultVectors)
                Tcip = icp.icp_transformation_matrix(motionVectors)
                Tego, _, _ = icp.icp_ego_motion_matrix(motionVectors)

                if not hasattr(self, "translation_history"):
                    self.translation_history = []
                plot4(plot_item, Tego, self.translation_history)

# ---------------- Plots ----------------
def plot1(plot_widget, clusters, predictions):
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud")
    for cid, data in clusters.items():
        clusterPoints = data['points']
        clusterDoppler = data['doppler_avg']
        clusterHits = data.get('hits', 0)
        if clusterPoints.shape[0] > 0:
            scatter = pg.ScatterPlotItem(
                x=clusterPoints[:, 0], y=clusterPoints[:, 1], size=8,
                pen=None, brush=pg.mkBrush(0, 200, 0, 150)
            )
            plot_widget.addItem(scatter)
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
    for cid, (px, py, *_) in predictions.items():
        pred_sc = pg.ScatterPlotItem(
            x=[px], y=[py], symbol='x', size=14,
            pen=pg.mkPen('r', width=2)
        )
        plot_widget.addItem(pred_sc)
        lbl = pg.TextItem(f"Pred {cid}", color='r')
        lbl.setPos(px+0.3, py+0.3)
        plot_widget.addItem(lbl)

def plot2(plot_widget, clusters, predictions):
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud")
    for cid, data in clusters.items():
        clusterPoints = data['points']
        clusterDoppler = data['doppler_avg']
        clusterHits = data.get('hits', 0)
        if clusterPoints.shape[0] > 0:
            scatter = pg.ScatterPlotItem(
                x=clusterPoints[:, 0], y=clusterPoints[:, 1], size=8,
                pen=None, brush=pg.mkBrush(0, 200, 0, 150)
            )
            plot_widget.addItem(scatter)
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
    for cid, (px, py, *_) in predictions.items():
        pred_sc = pg.ScatterPlotItem(
            x=[px], y=[py], symbol='x', size=14,
            pen=pg.mkPen('r', width=2)
        )
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

    mat_text = (
        f"Rᵀ =\n"
        f"[{Rt[0,0]:.3f} {Rt[0,1]:.3f}]\n"
        f"[{Rt[1,0]:.3f} {Rt[1,1]:.3f}]"
    )
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
        scatter = pg.ScatterPlotItem(
            x=xs, y=ys, symbol='x', size=10,
            pen=pg.mkPen('b', width=2)
        )
        plot_widget.addItem(scatter)

    txt = pg.TextItem(f"Translation:\nX={tx:.2f}, Y={ty:.2f}", color='b')
    txt.setPos(tx, ty)
    plot_widget.addItem(txt)

# ---------------- Main ----------------
if __name__ == "__main__":
    if VISUALIZATION_MODE == 1:
        tcp_thread = threading.Thread(target=tcp_reader_nonblocking, daemon=True)
        tcp_thread.start()
    else:
        logging.info("[INFO] Using log files for visualization...")

    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
