import socket
import threading
import queue
import numpy as np
import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pointFilter
import dbCluster
from collections import defaultdict

# -------------------------------------------------------------
# PARAMETERS
# -------------------------------------------------------------
FILTER_SNR_MIN   = 12
FILTER_Z_MIN     = -2
FILTER_Z_MAX     = 2
FILTER_Y_MIN     = 0.6
FILTER_Y_MAX     = 15
FILTER_PHI_MIN   = -85
FILTER_PHI_MAX   = 85
FILTER_DOPPLER_MIN = 0.0
FILTER_DOPPLER_MAX = 8.0

GRID_SPACING = 1.0  # meters

cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=2)

data_queue = queue.Queue()

# -------------------------------------------------------------
# TCP READER THREAD
# -------------------------------------------------------------
def tcp_reader():
    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp.connect(("192.168.63.97", 9000))
    tcp_file = tcp.makefile('r')
    print("[INFO] TCP Connected!")
    while True:
        line = tcp_file.readline()
        if not line:
            break
        data_queue.put(line.strip())

reader_thread = threading.Thread(target=tcp_reader, daemon=True)
reader_thread.start()

# -------------------------------------------------------------
# PARSERS
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
# PYQTGRAPH SETUP
# -------------------------------------------------------------
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Live Radar & IMU Viewer")
win.setBackground('w')  # ✅ whole canvas white

# === Panel 1: Raw radar points with Doppler ===
plot_raw = win.addPlot(row=0, col=0, title="Raw Points with Doppler")
plot_raw.getViewBox().setBackgroundColor('w')
plot_raw.setXRange(-10, 10)
plot_raw.setYRange(-10, 10)
plot_raw.showGrid(x=True, y=True, alpha=0.3)
plot_raw.getAxis('bottom').setTickSpacing(levels=[(GRID_SPACING, 0)])
plot_raw.getAxis('left').setTickSpacing(levels=[(GRID_SPACING, 0)])

scatter_raw = pg.ScatterPlotItem(pen=None, brush='k', size=8)
plot_raw.addItem(scatter_raw)
text_items = []

# === Panel 2: Filtered radar cloud ===
plot_filtered = win.addPlot(row=0, col=1, title="Filtered Radar Points (X,Y)")
plot_filtered.getViewBox().setBackgroundColor('w')
plot_filtered.setXRange(-10, 10)
plot_filtered.setYRange(-10, 10)
plot_filtered.showGrid(x=True, y=True, alpha=0.3)
plot_filtered.getAxis('bottom').setTickSpacing(levels=[(GRID_SPACING, 0)])
plot_filtered.getAxis('left').setTickSpacing(levels=[(GRID_SPACING, 0)])

scatter_filtered = pg.ScatterPlotItem(pen=None, brush='k', size=8)
plot_filtered.addItem(scatter_filtered)


# === Panel 3: IMU Heading with 2 arrows ===
plot_imu = win.addPlot(row=1, col=0, title="IMU Heading")
plot_imu.getViewBox().setBackgroundColor('w')
plot_imu.setXRange(-1, 1)
plot_imu.setYRange(-1, 1)
arrow_x = pg.ArrowItem(angle=0, pen='r', brush='r')
arrow_y = pg.ArrowItem(angle=90, pen='b', brush='b')
plot_imu.addItem(arrow_x)
plot_imu.addItem(arrow_y)

# === Panel 4: Averaged IMU info ===
plot_imu_text = win.addPlot(row=1, col=1, title="Averaged IMU (per frame)")
plot_imu_text.getViewBox().setBackgroundColor('w')
plot_imu_text.hideAxis('left')
plot_imu_text.hideAxis('bottom')
imu_text_item = pg.TextItem("", anchor=(0,0), color='k')
plot_imu_text.addItem(imu_text_item)


# -------------------------------------------------------------
# LIVE UPDATE LOOP
# -------------------------------------------------------------
raw_points = []
imu_samples = []
frame_count = 0

def update():
    global raw_points, imu_samples, frame_count, text_items

    while not data_queue.empty():
        line = data_queue.get()
        if line.startswith("[RADAR]"):
            raw_points.append(parse_radar(line))
        elif line.startswith("[IMU]"):
            imu_samples.append(parse_imu(line))

    if len(raw_points) < 1:
        return

    frame_count += 1
    if frame_count % 5 != 0:
        return

    # === FILTER & CLUSTER ===
    pts = raw_points
    pts = pointFilter.filterSNRmin(pts, FILTER_SNR_MIN)
    pts = pointFilter.filterCartesianZ(pts, FILTER_Z_MIN, FILTER_Z_MAX)
    pts = pointFilter.filterCartesianY(pts, FILTER_Y_MIN, FILTER_Y_MAX)
    pts = pointFilter.filterSphericalPhi(pts, FILTER_PHI_MIN, FILTER_PHI_MAX)
    pts = pointFilter.filterDoppler(pts, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

    c1, _ = cluster_processor_stage1.cluster_points(np.array([[p['x'], p['y'], p['z'], p['doppler']] for p in pts]))
    dense_points = np.vstack([c['points'] for c in c1.values()]) if c1 else np.empty((0,4))
    c2, _ = cluster_processor_stage2.cluster_points(dense_points) if len(dense_points) > 0 else ({}, None)

    scatter_filtered.setData([p['x'] for p in pts], [p['y'] for p in pts])

    # === IMU 2-axis ===
    if imu_samples:
        avg_quat = np.mean([imu['quat'] for imu in imu_samples], axis=0)
        yaw = compute_yaw({'quat': avg_quat})
        vx = math.cos(yaw)
        vy = math.sin(yaw)
        arrow_x.setStyle(angle=0)
        arrow_y.setStyle(angle=90)
        arrow_x.setPos(0, 0)
        arrow_y.setPos(0, 0)
        arrow_x.setStyle(angle=np.degrees(math.atan2(0, vx)) if vx != 0 else 0)
        arrow_y.setStyle(angle=np.degrees(math.atan2(vy, 0)) if vy != 0 else 90)

        text = f"Avg Quaternion:\n[{', '.join(f'{q:.3f}' for q in avg_quat)}]"
        imu_text_item.setText(text)

    # === Raw cloud with Doppler labels ===
    scatter_raw.setData([p['x'] for p in raw_points], [p['y'] for p in raw_points])
    for t in text_items:
        plot_raw.removeItem(t)
    text_items.clear()
    for p in raw_points:
        t = pg.TextItem(f"{p['doppler']:.2f}", color='r', anchor=(0,0))
        t.setPos(p['x'], p['y'])
        plot_raw.addItem(t)
        text_items.append(t)

    raw_points.clear()
    imu_samples.clear()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

print("[INFO] Running PyQtGraph app loop...")
app.exec_()
