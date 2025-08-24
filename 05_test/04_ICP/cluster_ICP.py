import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import sys

# ----------------------------
# Point-to-Point ICP
# ----------------------------
def point_to_point_icp(A, B, max_iterations=20, tolerance=1e-6):
    src = A.copy()
    dst = B.copy()
    prev_error = float('inf')
    cumulative_R = np.eye(2)
    cumulative_t = np.zeros(2)

    for _ in range(max_iterations):
        distances = np.linalg.norm(src[:, None, :] - dst[None, :, :], axis=2)
        indices = np.argmin(distances, axis=1)
        matched_dst = dst[indices]

        centroid_src = np.mean(src, axis=0)
        centroid_dst = np.mean(matched_dst, axis=0)
        AA = src - centroid_src
        BB = matched_dst - centroid_dst
        H = AA.T @ BB
        U, _, VT = np.linalg.svd(H)
        R = VT.T @ U.T
        if np.linalg.det(R) < 0:
            VT[1, :] *= -1
            R = VT.T @ U.T
        t = centroid_dst - R @ centroid_src

        src = (R @ src.T).T + t
        cumulative_R = R @ cumulative_R
        cumulative_t = R @ cumulative_t + t

        mean_error = np.mean(np.linalg.norm(src - matched_dst, axis=1))
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return cumulative_R, cumulative_t

# ----------------------------
# Simulate Cluster Motion
# ----------------------------
def generate_clusters(num_frames=40, num_clusters=6):
    np.random.seed(0)
    base = []
    for i in range(num_clusters):
        cx, cy = 3 * i, 2 * (i % 3)
        pts = np.random.randn(15, 2) * 0.3 + np.array([cx, cy])
        base.append(pts)
    frames = []
    for t in range(num_frames):
        angle = np.deg2rad(0.4 * t)
        translation = np.array([0.04 * t, 0.08 * t + 0.005 * t**1.5])
        R = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle),  np.cos(angle)]])
        clusters = []
        for pts in base:
            moved = (R @ pts.T).T + translation
            clusters.append(moved)
        frames.append(clusters)
    return frames

# ----------------------------
# Main Viewer
# ----------------------------
class PerClusterICPViewer(QtWidgets.QWidget):
    def __init__(self, frames):
        super().__init__()
        self.setWindowTitle("Per-Cluster ICP Ego-Motion Estimation")
        self.resize(1000, 700)
        layout = QtWidgets.QVBoxLayout(self)

        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.setXRange(-5, 60)
        self.plot.setYRange(-5, 60)
        self.plot.showGrid(x=True, y=True)
        layout.addWidget(self.plot)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(1)
        self.slider.setMaximum(len(frames) - 1)
        self.slider.valueChanged.connect(self.update_frame)
        layout.addWidget(self.slider)

        self.label = QtWidgets.QLabel("Frame: 1")
        layout.addWidget(self.label)

        self.frames = frames
        self.poses = [np.eye(3)]
        self.ego_path = [np.array([0.0, 0.0])]

        self.prev_points = pg.ScatterPlotItem(pen=None, brush='b', size=6)
        self.curr_points = pg.ScatterPlotItem(pen=None, brush='g', size=6)
        self.ego_curve = pg.PlotCurveItem(pen=pg.mkPen('r', width=2))
        self.plot.addItem(self.prev_points)
        self.plot.addItem(self.curr_points)
        self.plot.addItem(self.ego_curve)

        self.update_frame(1)

    def update_frame(self, idx):
        self.label.setText(f"Frame: {idx}")
        prev_clusters = self.frames[idx - 1]
        curr_clusters = self.frames[idx]

        Rs = []
        ts = []
        for prev_pts, curr_pts in zip(prev_clusters, curr_clusters):
            R, t = point_to_point_icp(curr_pts, prev_pts)
            Rs.append(R)
            ts.append(t)

        R_avg = np.mean(Rs, axis=0)
        t_avg = np.mean(ts, axis=0)

        R_inv = R_avg.T
        t_inv = -R_inv @ t_avg
        T = np.eye(3)
        T[:2, :2] = R_inv
        T[:2, 2] = t_inv
        new_pose = self.poses[-1] @ T
        self.poses.append(new_pose)
        self.ego_path.append(new_pose[:2, 2])

        P1 = np.vstack(prev_clusters)
        P2 = np.vstack(curr_clusters)
        self.prev_points.setData(P1[:, 0], P1[:, 1])
        self.curr_points.setData(P2[:, 0], P2[:, 1])
        path = np.array(self.ego_path)
        self.ego_curve.setData(path[:, 0], path[:, 1])

# ----------------------------
# Launch App
# ----------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    frames = generate_clusters()
    viewer = PerClusterICPViewer(frames)
    viewer.show()
    sys.exit(app.exec_())
