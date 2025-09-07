import sys
import socket
import struct
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout
from PyQt5.QtCore import QTimer


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
        "R": yaw_rotation_matrix(30),
        "T": np.array([-0.29, 0.0, 0.15])
    },
    "right": {
        "R": yaw_rotation_matrix(-30),
        "T": np.array([0.29, 0.0, 0.15])
    }
}

def transform_pointcloud(points, sensor_label):
    R = TRANSFORMATIONS[sensor_label]["R"]
    T = TRANSFORMATIONS[sensor_label]["T"]
    result = []

    for p in points:
        local = np.array([p['x'], p['y'], p['z']], dtype=float)
        global_coords = np.dot(R, local) + T

        p_new = p.copy()
        p_new['x'], p_new['y'], p_new['z'] = global_coords
        p_new['source'] = sensor_label
        result.append(p_new)

    return result


class RadarPlotter(QWidget):
    def __init__(self, host="127.0.0.1", port=9000):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.sock.setblocking(False)

        self.left_pts_raw = []
        self.right_pts_raw = []
        self.left_pts_trans = []
        self.right_pts_trans = []

        self.initUI()

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_data)
        self.timer.start(30)

    def initUI(self):
        layout = QHBoxLayout()

        # Raw
        self.raw_plot = pg.PlotWidget(title="Raw Point Cloud")
        self.raw_plot.setAspectLocked(True)
        self.raw_plot.showGrid(x=True, y=True)
        self.raw_left = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(0, 0, 255, 120), size=5)
        self.raw_right = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(255, 0, 0, 120), size=5)
        self.raw_plot.addItem(self.raw_left)
        self.raw_plot.addItem(self.raw_right)

        # Transformed
        self.trans_plot = pg.PlotWidget(title="Transformed Point Cloud")
        self.trans_plot.setAspectLocked(True)
        self.trans_plot.showGrid(x=True, y=True)
        self.trans_left = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(0, 0, 255, 120), size=5)
        self.trans_right = pg.ScatterPlotItem(pen=None, brush=pg.mkBrush(255, 0, 0, 120), size=5)
        self.trans_plot.addItem(self.trans_left)
        self.trans_plot.addItem(self.trans_right)

        layout.addWidget(self.raw_plot)
        layout.addWidget(self.trans_plot)
        self.setLayout(layout)

    def read_data(self):
        try:
            while True:
                header = self.sock.recv(4)
                if not header:
                    break
                packet_size = struct.unpack('I', header)[0]

                payload = b''
                while len(payload) < packet_size:
                    chunk = self.sock.recv(packet_size - len(payload))
                    if not chunk:
                        break
                    payload += chunk

                if len(payload) != packet_size:
                    print("[WARN] Incomplete packet")
                    break

                source_flag = payload[0:1].decode()  # 'L' or 'R'
                count = struct.unpack('I', payload[1:5])[0]
                offset = 5
                raw_points = []

                for _ in range(count):
                    x, y, z, doppler = struct.unpack('ffff', payload[offset:offset + 16])
                    raw_points.append({'x': x, 'y': y, 'z': z, 'doppler': doppler})
                    offset += 16

                if source_flag == 'L':
                    self.left_pts_raw = [(p['x'], p['y']) for p in raw_points]
                    transformed = transform_pointcloud(raw_points, "left")
                    self.left_pts_trans = [(p['x'], p['y']) for p in transformed]
                elif source_flag == 'R':
                    self.right_pts_raw = [(p['x'], p['y']) for p in raw_points]
                    transformed = transform_pointcloud(raw_points, "right")
                    self.right_pts_trans = [(p['x'], p['y']) for p in transformed]

        except BlockingIOError:
            pass

        self.raw_left.setData(pos=np.array(self.left_pts_raw))
        self.raw_right.setData(pos=np.array(self.right_pts_raw))
        self.trans_left.setData(pos=np.array(self.left_pts_trans))
        self.trans_right.setData(pos=np.array(self.right_pts_trans))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    viewer = RadarPlotter()
    viewer.setWindowTitle("Radar Viewer - Raw and Transformed")
    viewer.resize(1200, 600)
    viewer.show()
    sys.exit(app.exec_())
