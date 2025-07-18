import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from ClusterTracker import ClusterTracker
from frameAggregator import FrameAggregator
import pointFilter
import dbCluster

from fileSearch import find_project_root
from decodeFile import ImuCSVReader
from decodeFile import RadarCSVReader

# -------------------------------------------------------------
# CONSTANTS: Simulation parameters
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 5
FILTER_SNR_MIN = 12
FILTER_Z_MIN, FILTER_Z_MAX = -2, 2
FILTER_Y_MIN, FILTER_Y_MAX = 0.6, 15
FILTER_PHI_MIN, FILTER_PHI_MAX = -85, 85
FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX = 0.01, 8.0

# -------------------------------------------------------------
# OBJECTS: Processing stages
# -------------------------------------------------------------
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=2)


# -------------------------------------------------------------
# ENTRY POINT: Load data.
# -------------------------------------------------------------
radarLoader = RadarCSVReader(
    file_name="radar_straightWall_1.csv",
    folder_name="04_Logs-10072025_v2"
)
imuLoader = ImuCSVReader(
    file_name="imu_straightWall_1.csv",
    folder_name="04_Logs-10072025_v2"
)

imu_frames   = imuLoader.load_all()
radar_frames = radarLoader.load_all()

# Simulated radar_frames
class RadarPoint:
    def __init__(self, x, y, z, doppler, snr, noise):
        self.x = x
        self.y = y
        self.z = z
        self.doppler = doppler
        self.snr = snr
        self.noise = noise


class ClusterViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Radar Cluster Visualizer (6 Panels)")
        self.resize(1400, 800)

        self.frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
        self.tracker = ClusterTracker()

        self.labels = ["Raw", "Filtered", "Stage1 Clusters", "Stage2 Clusters", "Final Cluster", "Reserved6"]
        self.plots = {}
        self.layout = QtWidgets.QGridLayout()
        container = QtWidgets.QWidget()
        container.setLayout(self.layout)

        for i in range(6):
            pw = pg.PlotWidget(title=self.labels[i])
            pw.setXRange(-5, 5)
            pw.setYRange(0, 10)
            pw.showGrid(x=True, y=True)
            self.layout.addWidget(pw, i // 3, i % 3)
            self.plots[f"plot{i+1}"] = pw

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(len(radar_frames) - 1)
        self.slider.valueChanged.connect(self.on_slider_changed)

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(container)
        main_layout.addWidget(self.slider)

        main_widget = QtWidgets.QWidget()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        self.on_slider_changed(0)

    def on_slider_changed(self, frame_idx):
        self.frame_aggregator.clearBuffer()
        for offset in range(FRAME_AGGREGATOR_NUM_PAST_FRAMES + 1):
            # TODO: Here to add IMU
            idx = max(0, frame_idx - offset)
            self.frame_aggregator.updateBuffer(radar_frames[idx])
        raw_pc = self.frame_aggregator.getPoints()
        self.draw_points(self.plots["plot1"], raw_pc)

        pointCloud = pointFilter.filterSNRmin(raw_pc, FILTER_SNR_MIN)
        pointCloud = pointFilter.filterCartesianZ(pointCloud, FILTER_Z_MIN, FILTER_Z_MAX)
        pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)
        pointCloud = pointFilter.filterSphericalPhi(pointCloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filtered_pointCloud = pointFilter.filterDoppler(pointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)
        self.draw_points(self.plots["plot2"], filtered_pointCloud, label="Filtered")

        pc_for_clustering = pointFilter.extract_points(filtered_pointCloud)
        clusters_stage1, _ = cluster_processor_stage1.cluster_points(pc_for_clustering)
        self.draw_clusters(self.plots["plot3"], clusters_stage1)

        if len(clusters_stage1) > 0:
            points_stage1_flat = np.concatenate([clusteredData['points'] for clusteredData in clusters_stage1.values()], axis=0)
            clusters_stage2, _ = cluster_processor_stage2.cluster_points(points_stage1_flat)

            # Check for euclidean nearest cluster
            clusters_stage2 = self.tracker.associate_new_clusters(clusters_stage2)

            self.tracker.update(clusters_stage2)
            # Get active tracks
            clusters_stage2 = self.tracker.get_active_tracks()
            # Add predicted clusters from tracker for those that were not matched
            predicted_clusters = self.tracker.get_predicted_clusters()
            for cid, pred_data in predicted_clusters.items():
                if cid not in clusters_stage2:
                    clusters_stage2[cid] = pred_data  # Add predicted cluster to be drawn
            
            self.draw_clusters(self.plots["plot4"], clusters_stage2, tracked=True)

            finalCluster = dict(clusters_stage2)  # make a copy
            predicted_clusters = self.tracker.get_predicted_clusters()
            for cid, pred_data in predicted_clusters.items():
                if cid not in finalCluster:
                    finalCluster[cid] = pred_data

            if len(clusters_stage1) > 0:
                points_stage1_flat = np.concatenate([clusteredData['points'] for clusteredData in clusters_stage1.values()], axis=0)
                clusters_stage2, _ = cluster_processor_stage2.cluster_points(points_stage1_flat)

                # Check for euclidean nearest cluster
                clusters_stage2 = self.tracker.associate_new_clusters(clusters_stage2)

                self.tracker.update(clusters_stage2)
                # Get active tracks
                clusters_stage2 = self.tracker.get_active_tracks()
                # Add predicted clusters from tracker for those that were not matched
                predicted_clusters = self.tracker.get_predicted_clusters()
                for cid, pred_data in predicted_clusters.items():
                    if cid not in clusters_stage2:
                        clusters_stage2[cid] = pred_data  # Add predicted cluster to be drawn
                
                self.draw_clusters(self.plots["plot4"], clusters_stage2, tracked=True)

            finalCluster = dict(clusters_stage2)  # make a copy
            predicted_clusters = self.tracker.get_predicted_clusters()
            for cid, pred_data in predicted_clusters.items():
                if cid not in finalCluster:
                    finalCluster[cid] = pred_data

            for cid, data in finalCluster.items():
                points = data.get('points')
                if points is None or len(points) < 2:
                    continue

                x = points[:, 0]
                y = points[:, 1]
                dopplers = points[:, 3]

                phis = np.arctan2(x, y)
                A = np.stack([np.cos(phis), np.sin(phis)], axis=1)
                R = dopplers.reshape(-1, 1)

                try:
                    V, residuals, rank, s = np.linalg.lstsq(A, R, rcond=None)
                    v_x, v_y = V.flatten()
                    data['vx'] = float(v_x)
                    data['vy'] = float(v_y)
                    data['residual'] = float(residuals[0]) if residuals.size > 0 else 0.0
                except Exception as e:
                    print(f"[Warning] Velocity estimation failed for cluster {cid}: {e}")
                    data['vx'] = 0.0
                    data['vy'] = 0.0
                    data['residual'] = -1.0

                # ------------------------------
                # Step 1: Get historical velocity from tracker
                # ------------------------------
                track = self.tracker.tracks.get(cid)
                if track and len(track.history) >= 2:
                    c0 = track.history[-2]['centroid']
                    c1 = track.history[-1]['centroid']
                    vx_hist = c1[0] - c0[0]
                    vy_hist = c1[1] - c0[1]
                    # Store historical velocities
                    delta_frames = track.history[-1]['frame_idx'] - track.history[-2]['frame_idx']
                    if delta_frames > 0:
                        vx_hist = (c1[0] - c0[0]) / delta_frames
                        vy_hist = (c1[1] - c0[1]) / delta_frames
                    else:
                        vx_hist = vy_hist = 0.0

                    # ------------------------------
                    # Step 2: Compute error
                    # ------------------------------
                    vx_doppler = data['vx']
                    vy_doppler = data['vy']
                    error = np.linalg.norm([vx_doppler - vx_hist, vy_doppler - vy_hist])
                    data['vx_error'] = error

                    # Optional: Correct if error is large
                    if error > 1.0:  # you can tweak this threshold
                        data['vx'] = vx_hist
                        data['vy'] = vy_hist
                        data['corrected'] = True
                    else:
                        data['corrected'] = False
                else:
                    # If not enough history, just flag
                    data['vx_hist'] = 0.0
                    data['vy_hist'] = 0.0
                    data['vx_error'] = -1.0
                    data['corrected'] = False
            
            self.draw_clusters(self.plots["plot5"], finalCluster, tracked=True)



    def draw_points(self, plot, point_data, label=""):
        plot.clear()
        if not point_data: return
        pts = np.array([[p["x"], p["y"]] for p in point_data])
        scatter = pg.ScatterPlotItem(x=pts[:, 0], y=pts[:, 1], size=5,
                                     pen=None, brush=pg.mkBrush(180, 180, 255, 180))
        plot.addItem(scatter)
        cx, cy = np.mean(pts[:, 0]), np.mean(pts[:, 1])
        txt = pg.TextItem(text=label, color='w')
        txt.setPos(cx + 0.2, cy + 0.2)
        plot.addItem(txt)

    def draw_clusters(self, plot, cluster_data, tracked=False):
        plot.clear()
        for cid, data in cluster_data.items():
            pts = data['points']

            # Plot the points
            scatter = pg.ScatterPlotItem(x=pts[:, 0], y=pts[:, 1], size=6,
                                        pen=None, brush=pg.mkBrush(100 + cid * 40, 150, 255))
            plot.addItem(scatter)

            # Get centroid
            centroid = data['centroid']
            cx, cy = centroid[0], centroid[1]

            # Use the actual estimated values
            vx = data.get("vx", 0.0)
            vy = data.get("vy", 0.0)
            doppler = data.get("doppler_avg", 0.0)
            residual = data.get("residual", 0.0)

            # Construct text label
            error = data.get("vx_error", -1.0)
            vx_hist = data.get("vx_hist", 0.0)
            vy_hist = data.get("vy_hist", 0.0)
            corrected = data.get("corrected", False)

            text = f"ID {cid}\nD={doppler:.2f}\n"
            text += f"vx={vx:.2f}, vy={vy:.2f}\n"
            text += f"hx={vx_hist:.2f}, hy={vy_hist:.2f}\n"
            text += f"err={error:.2f}"
            if corrected:
                text += " (CORR)"


            # Add missed counter info if tracked
            if tracked:
                track = self.tracker.tracks.get(cid)
                if track:
                    text += f"\nMissed={track.missing_counter}"

            # Plot text label next to the cluster
            label = pg.TextItem(text=text, anchor=(0, 0), color='w')
            label.setPos(cx + 0.5, cy + 0.3)
            plot.addItem(label)

            # Optional: cross marker for tracked centroids
            if tracked:
                cross = pg.TextItem(text="X", color='r')
                cross.setPos(cx, cy)
                plot.addItem(cross)



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
