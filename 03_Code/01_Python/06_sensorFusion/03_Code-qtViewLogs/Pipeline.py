import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider
)
from PyQt5.QtCore import Qt

from ClusterTracker import ClusterTracker
from decodeFile import ImuCSVReader, RadarCSVReader
import helper
from frameAggregator import FrameAggregator
import dbCluster
import pointFilter

# -------------------------------------------------------------
# PARAMETERS
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 10
FILTER_SNR_MIN                  = 12
FILTER_Z_MIN, FILTER_Z_MAX      = -2, 2
FILTER_Y_MIN, FILTER_Y_MAX      = 0.6, 15
FILTER_PHI_MIN, FILTER_PHI_MAX  = -85, 85
FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX = 0.0, 8.0
# -------------------------------------------------------------

# Instantiate readers and global aggregators
radarLoader       = RadarCSVReader("radar_straightWall_1.csv", "04_Logs-10072025_v2")
imuLoader         = ImuCSVReader(  "imu_straightWall_1.csv",   "04_Logs-10072025_v2")
_radarAgg         = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_imuAgg           = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

# Two‐stage DBSCAN processors
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=2)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=2)

# ------------------------------
# Synthetic data generator + filters (unchanged)
# ------------------------------
def generate_raw_data(frame_idx):
    clusters = {}
    for cid in range(3):
        x = cid * 3 + frame_idx * 0.2
        y = cid * 2 + frame_idx * 0.15
        z = 0
        pts = np.random.randn(20,3)*0.4 + [x,y,z]
        clusters[cid] = {'centroid':[x,y,z],'points':pts,'doppler':np.random.uniform(-1,1)}
    return clusters

def apply_snr_filter(clusters):
    return {
        cid:{**c,'points':c['points'][np.linalg.norm(c['points'],axis=1)>0.5]}
        for cid,c in clusters.items()
    }

def apply_z_filter(clusters):
    return {
        cid:{**c,'points':c['points'][(c['points'][:,2]>-0.5)&(c['points'][:,2]<0.5)]}
        for cid,c in clusters.items()
    }

def apply_y_filter(clusters):
    return {
        cid:{**c,'points':c['points'][(c['points'][:,1]>1)&(c['points'][:,1]<10)]}
        for cid,c in clusters.items()
    }

def apply_phi_filter(clusters):
    return {
        cid:{**c,'points':c['points'][np.abs(np.arctan2(c['points'][:,1],c['points'][:,0]))<np.pi/4]}
        for cid,c in clusters.items()
    }

def apply_doppler_filter(clusters):
    return {
        cid:c for cid,c in clusters.items() if -0.3 < c['doppler'] < 0.3
    }

# ------------------------------
# Plot-1’s custom view (unchanged)
# ------------------------------
def plot1(plot_widget, clusters, predictions):
    plot_widget.clear()
    plot_widget.setTitle("Custom Plot 1 View")
    for cid,data in clusters.items():
        pts = data['points']
        if pts.shape[0]>0:
            scatter = pg.ScatterPlotItem(
                x=pts[:,0], y=pts[:,1], size=8,
                pen=None, brush=pg.mkBrush(0,200,0,150)
            )
            plot_widget.addItem(scatter)
        # unpack centroid (x,y ignore others)
        cx, cy = data['centroid'][:2]
        label = pg.TextItem(f"ID {cid}", anchor=(0.5,-0.2), color='w')
        label.setPos(cx, cy)
        plot_widget.addItem(label)
    for cid,(px,py, *_ ) in predictions.items():
        pred_sc = pg.ScatterPlotItem(
            x=[px], y=[py], symbol='x', size=14,
            pen=pg.mkPen('r',width=2)
        )
        plot_widget.addItem(pred_sc)
        lbl = pg.TextItem(f"Pred {cid}", color='r')
        lbl.setPos(px+0.3, py+0.3)
        plot_widget.addItem(lbl)

# ------------------------------
# Main Viewer
# ------------------------------
class ClusterViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("6-Panel Cluster Viewer")
        self.resize(1200,900)

        # Load data once
        self.imu_frames   = imuLoader.load_all()
        self.radar_frames = radarLoader.load_all()
        self.radarDataSetLength = len(self.radar_frames)
        self.currentFrame = -1

        # One tracker per plot
        self.trackers = {f"plot{i}": ClusterTracker() for i in range(1,7)}

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

        ctrl = QHBoxLayout()
        self.slider_label = QLabel("Frame: 0")
        self.slider       = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.radarDataSetLength-1)
        self.slider.valueChanged.connect(self.on_slider_changed)
        ctrl.addWidget(self.slider_label); ctrl.addWidget(self.slider)
        main_layout.addLayout(ctrl)

        # Start at frame 0
        self.on_slider_changed(0)

    def on_slider_changed(self, newFrame):
        # rewind on backward move
        if newFrame < self.currentFrame:
            _radarAgg.clearBuffer()
            _imuAgg.clearBuffer()
            self.currentFrame = -1

        # aggregate only new frames
        for f in range(self.currentFrame+1, newFrame+1):
            _radarAgg.updateBuffer(self.radar_frames[f])
            _imuAgg.updateBuffer(  self.imu_frames[f])
        self.currentFrame  = newFrame
        self.slider_label.setText(f"Frame: {newFrame}")
        self.update_all_plots()

    def update_all_plots(self):
        # ------------------------
        # FILTERING PIPELINE (Plot 1 only)
        # ------------------------
        raw_pc = _radarAgg.getPoints()  # list of dicts
        pc = pointFilter.filterSNRmin( raw_pc, FILTER_SNR_MIN)
        pc = pointFilter.filterCartesianZ(pc, FILTER_Z_MIN, FILTER_Z_MAX)
        pc = pointFilter.filterCartesianY(pc, FILTER_Y_MIN, FILTER_Y_MAX)
        pc = pointFilter.filterSphericalPhi(pc, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filtered_pc = pointFilter.filterDoppler(pc, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

        # ------------------------
        # STAGE 1 CLUSTERING
        # ------------------------
        pts1 = pointFilter.extract_points(filtered_pc)
        cluster_s1, _ = cluster_processor_stage1.cluster_points(pts1)

        # ------------------------
        # STAGE 2 CLUSTERING
        # ------------------------
        if len(cluster_s1)>0:
            pts_flat = np.vstack([cdata['points'] for cdata in cluster_s1.values()])
            clusters_p1, _ = cluster_processor_stage2.cluster_points(pts_flat)
        else:
            clusters_p1 = {}

        # now draw each plot
        for name, plot_item in self.plots.items():
            if name == "plot1":
                clusters = clusters_p1
            else:
                synth = generate_raw_data(self.currentFrame)
                if name=="plot2":
                    clusters = apply_snr_filter(synth)
                elif name=="plot3":
                    clusters = apply_z_filter(synth)
                elif name=="plot4":
                    clusters = apply_y_filter(synth)
                elif name=="plot5":
                    clusters = apply_phi_filter(synth)
                else:  # plot6
                    clusters = apply_doppler_filter(synth)

            tracker = self.trackers[name]
            tracker.update(clusters)
            preds = tracker.get_predictions()

            if name=="plot1":
                plot1(plot_item, clusters, preds)
            else:
                self.draw_plot(plot_item, clusters, preds)

    def draw_plot(self, plot_widget, detected, predicted):
        plot_widget.clear()
        for cid,data in detected.items():
            pts = data['points']
            if pts.shape[0]>0:
                scatter = pg.ScatterPlotItem(
                    x=pts[:,0], y=pts[:,1], size=6,
                    pen=None, brush=pg.mkBrush(100+cid*50,150,255)
                )
                plot_widget.addItem(scatter)
            cx, cy = data['centroid'][:2]
            dop = data.get('doppler',0.0)
            text = f"ID {cid}\nD={dop:.2f}"
            lbl = pg.TextItem(text,anchor=(0,0),color='w')
            lbl.setPos(cx+0.5, cy+0.3)
            plot_widget.addItem(lbl)
        for cid,(px,py, *_ ) in predicted.items():
            psc = pg.ScatterPlotItem(
                x=[px], y=[py], symbol='x', size=14,
                pen=pg.mkPen('r')
            )
            plot_widget.addItem(psc)
            lbl = pg.TextItem(f"Pred {cid}", color='r')
            lbl.setPos(px+0.3,py+0.3)
            plot_widget.addItem(lbl)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
