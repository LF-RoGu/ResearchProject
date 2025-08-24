import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import sys

# ----------------------------
# ICP Utility: Point-to-Point ICP (simplified)
# ----------------------------
def point_to_point_icp(A, B, max_iterations=20, tolerance=1e-6):
    src = A.copy()
    dst = B.copy()
    prev_error = float('inf')
    cumulative_R = np.eye(2)
    cumulative_t = np.zeros(2)

    for _ in range(max_iterations):
        # Nearest neighbor matching
        distances = np.linalg.norm(src[:, None, :] - dst[None, :, :], axis=2)
        indices = np.argmin(distances, axis=1)
        matched_dst = dst[indices]

        # Estimate transform using Kabsch algorithm
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

        # Apply transformation
        src = (R @ src.T).T + t

        # Accumulate transform
        cumulative_R = R @ cumulative_R
        cumulative_t = R @ cumulative_t + t

        mean_error = np.mean(np.linalg.norm(src - matched_dst, axis=1))
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return cumulative_R, cumulative_t

# ----------------------------
# Simulate 6 Static Clusters Moving w.r.t. Radar
# ----------------------------
def generate_cluster_sequence(num_frames=40, num_clusters=6):
    np.random.seed(42)
    base_clusters = []
    for i in range(num_clusters):
        cx, cy = 3.0 * i, 1.5 * (i % 3)
        points = np.random.randn(15, 2) * 0.3 + np.array([cx, cy])
        base_clusters.append(points)

    frames = []
    for t in range(num_frames):
        angle = np.deg2rad(0.3 * t)  # slight rotation
        translation = np.array([0.04 * t, 0.08 * t + 0.005 * t**1.5])
        R = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle),  np.cos(angle)]])
        transformed = []
        for cluster in base_clusters:
            moved = (R @ cluster.T).T + translation
            transformed.append(moved)
        frames.append(np.vstack(transformed))
    return frames

# ----------------------------
# PyQtGraph GUI
# ----------------------------
class ICPViewer(QtWidgets.QWidget):
    def __init__(self, frames):
        super().__init__()
        self.setWindowTitle("Global ICP Ego-Motion Estimation")
        self.resize(1000, 700)
        self.layout = QtWidgets.QVBoxLayout(self)

        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        self.plot.setXRange(-5, 60)
        self.plot.setYRange(-5, 60)
        self.layout.addWidget(self.plot)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(1)
        self.slider.setMaximum(len(frames) - 1)
        self.slider.valueChanged.connect(self.update_frame)
        self.layout.addWidget(self.slider)

        self.label = QtWidgets.QLabel("Frame: 1")
        self.layout.addWidget(self.label)

        self.frames = frames
        self.ego_path = [np.array([0.0, 0.0])]
        self.poses = [np.eye(3)]

        self.prev_points = pg.ScatterPlotItem(pen=None, brush='b', size=6)
        self.curr_points = pg.ScatterPlotItem(pen=None, brush='g', size=6)
        self.aligned_points = pg.ScatterPlotItem(pen=pg.mkPen('w'), brush=None, size=10, symbol='x')
        self.ego_curve = pg.PlotCurveItem(pen=pg.mkPen('r', width=2))
        self.plot.addItem(self.prev_points)
        self.plot.addItem(self.curr_points)
        self.plot.addItem(self.aligned_points)
        self.plot.addItem(self.ego_curve)

        self.update_frame(1)

    def update_frame(self, idx):
        self.label.setText(f"Frame: {idx}")
        P1 = self.frames[idx - 1]
        P2 = self.frames[idx]

        R, t = point_to_point_icp(P2, P1)
        R_inv = R.T
        t_inv = -R_inv @ t
        T = np.eye(3)
        T[:2, :2] = R_inv
        T[:2, 2] = t_inv
        new_pose = self.poses[-1] @ T
        self.poses.append(new_pose)
        self.ego_path.append(new_pose[:2, 2])

        aligned = (R @ P2.T).T + t
        self.prev_points.setData(P1[:, 0], P1[:, 1])
        self.curr_points.setData(P2[:, 0], P2[:, 1])
        self.aligned_points.setData(aligned[:, 0], aligned[:, 1])
        path = np.array(self.ego_path)
        self.ego_curve.setData(path[:, 0], path[:, 1])

# ----------------------------
# Run
# ----------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    frames = generate_cluster_sequence()
    viewer = ICPViewer(frames)
    viewer.show()
    sys.exit(app.exec_())
