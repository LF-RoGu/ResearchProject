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

CLUSTER_DEBUG = False

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
            idx = max(0, frame_idx - offset)
            self.frame_aggregator.updateBuffer(radar_frames[idx])
        raw_pc = self.frame_aggregator.getPoints()
        self.draw_points(self.plots["plot1"], raw_pc)

        # --- filtering & stage1/stage2 clustering (plots 2–4) as before ---
        pointCloud = pointFilter.filterSNRmin(raw_pc, FILTER_SNR_MIN)
        pointCloud = pointFilter.filterCartesianZ(pointCloud, FILTER_Z_MIN, FILTER_Z_MAX)
        pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)
        pointCloud = pointFilter.filterSphericalPhi(pointCloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filtered_pointCloud = pointFilter.filterDoppler(pointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)
        self.draw_points(self.plots["plot2"], filtered_pointCloud, label="Filtered")

        pc_for_clustering = pointFilter.extract_points(filtered_pointCloud)
        clusters_stage1, _ = cluster_processor_stage1.cluster_points(pc_for_clustering)
        self.draw_clusters(self.plots["plot3"], clusters_stage1)

        if clusters_stage1:
            pts_flat = np.concatenate([c['points'] for c in clusters_stage1.values()], axis=0)
            clusters_stage2, _ = cluster_processor_stage2.cluster_points(pts_flat)

            # TODO: stamp each cluster with current frame so linear regression can use it
            for cid, cdata in clusters_stage2.items():
                cdata['frame'] = frame_idx

            clusters_stage2 = self.tracker.associate_new_clusters(clusters_stage2)
            self.tracker.update(clusters_stage2, frame_idx)
            clusters_stage2 = self.tracker.get_active_tracks()
            for cid, centroid in self.tracker.get_predicted_clusters().items():
                if cid not in clusters_stage2:
                    clusters_stage2[cid] = {
                        'centroid': centroid,
                        'points': np.zeros((0, 4)),   # empty array so downstream code won’t break
                        # you can optionally carry over other fields (e.g. last doppler_avg)
                    }

            self.draw_clusters(self.plots["plot4"], clusters_stage2, tracked=True)

            # Build finalCluster with Doppler‐based vx,vy + residual
            finalCluster = dict(clusters_stage2)
            for cid, data in finalCluster.items():
                pts = data.get('points', None)
                if pts is None or len(pts) < 2:
                    data.update({'vx': 0.0, 'vy': 0.0, 'residual': 0.0})
                    continue

                x = pts[:,0]; y = pts[:,1]; dopplers = pts[:,3]
                phis = np.arctan2(x, y)
                A = np.stack([np.cos(phis), np.sin(phis)], axis=1)
                R = dopplers.reshape(-1,1)
                try:
                    V, residuals, *_ = np.linalg.lstsq(A, R, rcond=None)
                    v_flat = V.flatten()  # TODO: flatten before float conversion
                    data['vx'], data['vy'] = float(v_flat[0]), float(v_flat[1])
                    data['residual']      = float(residuals[0]) if residuals.size else 0.0
                except Exception as e:
                    print(f"[DEBUG] Doppler‐fit failed for {cid}: {e}")
                    data.update({'vx':0.0, 'vy':0.0, 'residual':-1.0})

                # TODO: compute linear‐regression slopes from history frames
                hist = self.tracker.tracks[cid].history
                if len(hist) >= 2:
                    frames = np.array([h['frame'] for h in hist])
                    xs     = np.array([h['centroid'][0] for h in hist])
                    ys     = np.array([h['centroid'][1] for h in hist])
                    slope_x, _ = np.polyfit(frames, xs, 1)
                    slope_y, _ = np.polyfit(frames, ys, 1)
                    data['lx'], data['ly'] = float(slope_x), float(slope_y)
                else:
                    data['lx'], data['ly'] = 0.0, 0.0

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
            scatter = pg.ScatterPlotItem(
                x=pts[:,0], y=pts[:,1], size=6,
                pen=None, brush=pg.mkBrush(100 + cid*40, 150, 255)
            )
            plot.addItem(scatter)

            # Centroid
            cx, cy = data['centroid'][:2]

            # use both Doppler‐based and linear‐regression velocities
            vx, vy = data.get('vx',0.0), data.get('vy',0.0)
            lx, ly = data.get('lx',0.0), data.get('ly',0.0)
            doppler = data.get('doppler_avg',0.0)
            res     = data.get('residual',0.0)

            if CLUSTER_DEBUG:
                print(
                    f"[Cluster {cid}]  "
                    f"centroid={data['centroid']}  "
                    f"points={len(data['points'])}  "
                    f"D={data.get('doppler_avg', 0):.2f}  "
                    f"vx={data['vx']:.2f}, vy={data['vy']:.2f}  "
                    f"lx={data['lx']:.2f}, ly={data['ly']:.2f}"
                )

            # Construct the label
            txt = (
                f"ID {cid}\n"
                f"D={doppler:.2f}\n"
                f"vx={vx:.2f}, vy={vy:.2f}\n"
                f"lx={lx:.2f}, ly={ly:.2f}\n"
                f"res={res:.2f}"
            )

            if tracked:
                miss = self.tracker.tracks[cid].missing_counter
                txt += f"\nMissed={miss}"

            label = pg.TextItem(text=txt, anchor=(0,0), color='w')
            label.setPos(cx + 0.5, cy + 0.3)
            plot.addItem(label)

            if tracked and self.tracker.tracks[cid].missing_counter > 0:
                cross = pg.TextItem(text="X", color='r')
                cross.setPos(cx, cy)
                plot.addItem(cross)





if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
