import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton
)
from PyQt5.QtCore import Qt

from ClusterTracker import ClusterTracker
from decodeFile import ImuCSVReader, RadarCSVReader
import helper
from frameAggregator import FrameAggregator
import dbCluster
import pointFilter
import icp

import pprint

# -------------------------------------------------------------
# PARAMETERS
# -------------------------------------------------------------
FRAME_AGGREGATOR_NUM_PAST_FRAMES = 10
FILTER_SNR_MIN                  = 12
FILTER_Z_MIN, FILTER_Z_MAX      = -2, 2
FILTER_Y_MIN, FILTER_Y_MAX      = 0.6, 15
FILTER_PHI_MIN, FILTER_PHI_MAX  = -85, 85
FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX = 0.01, 8.0
TRACKER_MAX_MISSES  = 10     # how many frames a track can disappear before deletion
TRACKER_DIST_THRESH = 0.5   # max movement (meters) allowed per frame for matching

# Global variables for current (P) and previous (Q) frame clusters
P = None  # clusters for current frame t
Q = None  # clusters for previous frame t-1
icp_history = {
    'result_vectors':    [],
    'motion_vectors':    [],
    'world_transforms':  [],
    'ego_transforms':    []
}

# Instantiate readers and global aggregators
radarLoader       = RadarCSVReader("radar_straightWall_1.csv", "05_Logs-10072025_v2")
imuLoader         = ImuCSVReader(  "imu_straightWall_1.csv",   "05_Logs-10072025_v2")
_radarAgg         = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
_imuAgg           = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)

# Two‐stage DBSCAN processors
cluster_processor_stage1 = dbCluster.ClusterProcessor(eps=2.0, min_samples=5)
cluster_processor_stage2 = dbCluster.ClusterProcessor(eps=1.0, min_samples=2)


def pretty_print_clusters(clusters, label="Clusters"):
    """
    Nicely prints a summary of `clusters` dict: ID, centroid, doppler, hits, and point count.
    """
    if clusters is None:
        print(f"{label}: None")
        return
    print(f"{label} ({len(clusters)} clusters):")
    for cid, data in clusters.items():
        centroid = data.get('centroid')
        dop = data.get('doppler_avg')
        hits = data.get('hits', None)
        missed = data.get('missed', None)
        pts = data.get('points')
        pt_count = len(pts) if pts is not None else 0
        print(f" - ID {cid}: centroid={centroid}, doppler={dop:.2f}, hits={hits}, missed={missed}, points={pt_count}")

# ------------------------------
# Plot-1’s custom view (unchanged)
# ------------------------------
def plot1(plot_widget, pointCloud):
    """
    Render only the filtered point cloud (pointCloud),
    which may be a list of {'x','y',…} dicts.
    """
    import pyqtgraph as pg

    # 1) Clear previous items and set title
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud - Filtered")

    # 2) Bail out if empty
    if not pointCloud:
        return

    # 3) Extract x/y coords from each dict
    x, y = [], []
    for p in pointCloud:
        # safely skip any malformed entries
        if isinstance(p, dict) and 'x' in p and 'y' in p:
            x.append(p['x'])
            y.append(p['y'])

    # 4) Plot if we got any points
    scatter = pg.ScatterPlotItem(
        x=x,
        y=y,
        size=8,
        pen=None,
        brush=pg.mkBrush(0, 200, 0, 150)
    )
    plot_widget.addItem(scatter)

    for p in pointCloud:
        if isinstance(p, dict) and 'x' in p and 'y' in p and 'doppler' in p:
            dop = p['doppler']
            txt = pg.TextItem(f"{dop:.2f}", color='y', anchor=(0,1))
            txt.setPos(p['x'] + 0.1, p['y'] + 0.1)
            plot_widget.addItem(txt)


# ------------------------------
# Plot-2’s custom view (unchanged)
# ------------------------------
def plot2(plot_widget, clusters, predictions):
    plot_widget.clear()
    plot_widget.setTitle("Point Cloud - Clustered")
    for cid,data in clusters.items():
        clusterPoints = data['points']
        clusterDoppler = data['doppler_avg']
        clusterHits    = data.get('hits', 0)
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
# Plot-3’s custom view (unchanged)
# ------------------------------
def plot3(plot_widget, ego_matrix):
    """
    Dedicated panel for visualizing ego-motion rotation.
    - Draws a yellow arrow showing rotation
    - Prints the 2x2 rotation matrix R^T
    """
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
# Main Viewer
# ------------------------------
class ClusterViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel Cluster Viewer")
        self.resize(1200,900)

        # Load data once
        self.imu_frames   = imuLoader.load_all()
        self.radar_frames = radarLoader.load_all()
        self.radarDataSetLength = len(self.radar_frames)
        self.currentFrame = -1

        # Plot will now use spatial matching with tuned parameters; others remain default
        self.trackers = {
            f"plot{i}": (
                ClusterTracker(
                    max_misses=TRACKER_MAX_MISSES,
                    dist_threshold=TRACKER_DIST_THRESH
                ) if i == 1 else ClusterTracker()
            )
            for i in range(1, 5)
        }

        # Build UI
        main_layout = QVBoxLayout(self)
        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget)
        self.plots = {}
        for idx in range(1,5):
            row, col = divmod(idx-1,3)
            p = self.plot_widget.addPlot(row=row,col=col)
            p.setXRange(-2,20); p.setYRange(-2,20)
            p.setAspectLocked(True); p.showGrid(x=True,y=True)
            p.setTitle(f"Plot {idx}")
            self.plots[f"plot{idx}"] = p

        
        # Control bar: label, slider, buttons
        ctrl = QHBoxLayout()
        self.slider_label = QLabel("Frame: 0")
        self.slider       = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.radarDataSetLength-1)
        self.slider.valueChanged.connect(self.on_slider_changed)

        self.prev_btn = QPushButton("<")
        self.next_btn = QPushButton(">")
        self.prev_btn.clicked.connect(lambda: self.change_slider(-1))
        self.next_btn.clicked.connect(lambda: self.change_slider(+1))

        ctrl.addWidget(self.slider_label)
        ctrl.addWidget(self.slider)
        ctrl.addWidget(self.prev_btn)
        ctrl.addWidget(self.next_btn)
        main_layout.addLayout(ctrl)

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
        # Call plots to be drawn
        self.update_all_plots()

    def change_slider(self, delta):
        """
        Move the frame slider by delta (±1) and clamp to [min, max].
        """
        new_val = self.slider.value() + delta
        new_val = max(self.slider.minimum(),
                      min(self.slider.maximum(), new_val))
        self.slider.setValue(new_val)


    def update_all_plots(self):
        global P, Q
        global icp_history
        # Filtern Pipeline information (Plot 1 only)
        rawPointCloud = _radarAgg.getPoints()  # list of dicts
        pointCloud = pointFilter.filterSNRmin( rawPointCloud, FILTER_SNR_MIN)
        pointCloud = pointFilter.filterCartesianZ(pointCloud, FILTER_Z_MIN, FILTER_Z_MAX)
        pointCloud = pointFilter.filterCartesianY(pointCloud, FILTER_Y_MIN, FILTER_Y_MAX)
        pointCloud = pointFilter.filterSphericalPhi(pointCloud, FILTER_PHI_MIN, FILTER_PHI_MAX)
        filteredPointCloud = pointFilter.filterDoppler(pointCloud, FILTER_DOPPLER_MIN, FILTER_DOPPLER_MAX)

        # Stage 1 clustering
        clusterProcessor_stage1 = pointFilter.extract_points(filteredPointCloud)
        clusterProcessor_stage1, _ = cluster_processor_stage1.cluster_points(clusterProcessor_stage1)

        # Stage 2 clustering
        if len(clusterProcessor_stage1)>0:
            clusterProcessor_flat = np.vstack([cdata['points'] for cdata in clusterProcessor_stage1.values()])
            clusterProcessor_final, _ = cluster_processor_stage2.cluster_points(clusterProcessor_flat)
        else:
            clusterProcessor_final = {}

        # Store value that was stored in P into Q
        #Q = P
        # Obtain the current cluster set of points
        #P = clusterProcessor_final


        # now draw each plot
        for name, plot_item in self.plots.items():

            if name == "plot1":

                plot1(plot_item, filteredPointCloud)
                
            if name == "plot2":
                # Plot 1: real clusters → spatial tracker 
                # update the tracker with the fresh Stage-2 clusters
                self.trackers[name].update(clusterProcessor_final)

                # pull out the tracks (persistent IDs) and their data
                clusters = self.trackers[name].get_active_tracks()

                # get predictions for any tracks that missed this frame
                preds = self.trackers[name].get_predictions()

                # TODO: Perform odometry calculation here
                for tid, trk_data in clusters.items():
                    history = trk_data['history']    # a list of np.array centroids
                    currentDoppler = trk_data['doppler_avg']
                    hits    = trk_data['hits']
                    missed  = trk_data['missed']

                plot2(plot_item, clusters, preds)
            if name == "plot3":
                # Plot 1: real clusters → spatial tracker 
                # update the tracker with the fresh Stage-2 clusters
                self.trackers[name].update(clusterProcessor_final)

                # pull out the tracks (persistent IDs) and their data
                clusters = self.trackers[name].get_active_tracks()

                # get predictions for any tracks that missed this frame
                preds = self.trackers[name].get_predictions()

                # Store value that was stored in P into Q
                Q = P
                # Obtain the current cluster set of points
                P = clusters

                #print(f"Current frame: {self.currentFrame}")
                #pretty_print_clusters(P, "[P] Current Clusters (Frame t)")
                #pretty_print_clusters(Q, "[Q] Previous Clusters (Frame t-1)")
                #print("-----------------------------------------------")

                resultVectors = icp.icp_translation_vector(P, Q)
                icp_history['result_vectors'].append(resultVectors)
                motionVectors = icp.icp_get_transformation_average(resultVectors)
                icp_history['motion_vectors'].append(motionVectors)
                worldMotion = icp.icp_transformation_matrix(motionVectors)
                icp_history['world_transforms'].append(worldMotion)
                Tego = icp.icp_ego_motion_matrix(motionVectors)
                icp_history['ego_transforms'].append(Tego)

                plot3(plot_item, Tego)
            if name == "plot4":
                # Plot 1: real clusters → spatial tracker 
                # update the tracker with the fresh Stage-2 clusters
                self.trackers[name].update(clusterProcessor_final)

                # pull out the tracks (persistent IDs) and their data
                clusters = self.trackers[name].get_active_tracks()

                # get predictions for any tracks that missed this frame
                preds = self.trackers[name].get_predictions()

                # Store value that was stored in P into Q
                Q = P
                # Obtain the current cluster set of points
                P = clusters

                resultVectors = icp.icp_translation_vector(P, Q)
                icp_history['result_vectors'].append(resultVectors)
                motionVectors = icp.icp_get_transformation_average(resultVectors)
                icp_history['motion_vectors'].append(motionVectors)
                worldMotion = icp.icp_transformation_matrix(motionVectors)
                icp_history['world_transforms'].append(worldMotion)
                Tego = icp.icp_ego_motion_matrix(motionVectors)
                icp_history['ego_transforms'].append(Tego)

                #print("Avg Translation:", motionVectors['translation_avg'])
                #print("Avg Rotation:", motionVectors['rotation_avg'])


                if not hasattr(self, "translation_history"):
                    self.translation_history = []
                plot4(plot_item, Tego, self.translation_history)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
