import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider
)
from PyQt5.QtCore import Qt
from ClusterTracker import ClusterTracker

# ------------------------------
# Simulation Config
# ------------------------------
NUM_FRAMES = 30


# ------------------------------
# Data Generators & Filters
# ------------------------------

def generate_raw_data(frame_idx):
    clusters = {}
    for cid in range(3):
        x = cid * 3 + frame_idx * 0.2
        y = cid * 2 + frame_idx * 0.15
        z = 0
        points = np.random.randn(20, 3) * 0.4 + [x, y, z]
        clusters[cid] = {
            'centroid': [x, y, z],
            'points': points,
            'doppler': np.random.uniform(-1, 1)
        }
    return clusters


def apply_snr_filter(clusters):
    return {
        cid: {
            **c,
            'points': c['points'][np.linalg.norm(c['points'], axis=1) > 0.5]
        }
        for cid, c in clusters.items()
    }


def apply_z_filter(clusters):
    return {
        cid: {
            **c,
            'points': c['points'][(c['points'][:, 2] > -0.5) & (c['points'][:, 2] < 0.5)]
        }
        for cid, c in clusters.items()
    }


def apply_y_filter(clusters):
    return {
        cid: {
            **c,
            'points': c['points'][(c['points'][:, 1] > 1) & (c['points'][:, 1] < 10)]
        }
        for cid, c in clusters.items()
    }


def apply_phi_filter(clusters):
    return {
        cid: {
            **c,
            'points': c['points'][np.abs(np.arctan2(c['points'][:, 1], c['points'][:, 0])) < np.pi / 4]
        }
        for cid, c in clusters.items()
    }


def apply_doppler_filter(clusters):
    return {
        cid: c for cid, c in clusters.items()
        if -0.3 < c['doppler'] < 0.3
    }


# ------------------------------
# Main Viewer Class
# ------------------------------

class ClusterViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("6-Panel Cluster Viewer")
        self.resize(1200, 900)

        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget)

        self.plots = {}
        self.trackers = {}

        # Prepare six different filtered datasets
        self.data_sets = {
            "plot1": [generate_raw_data(f) for f in range(NUM_FRAMES)],
            "plot2": [apply_snr_filter(generate_raw_data(f)) for f in range(NUM_FRAMES)],
            "plot3": [apply_z_filter(generate_raw_data(f)) for f in range(NUM_FRAMES)],
            "plot4": [apply_y_filter(generate_raw_data(f)) for f in range(NUM_FRAMES)],
            "plot5": [apply_phi_filter(generate_raw_data(f)) for f in range(NUM_FRAMES)],
            "plot6": [apply_doppler_filter(generate_raw_data(f)) for f in range(NUM_FRAMES)],
        }

        self.trackers = {plot: ClusterTracker() for plot in self.data_sets}

        # Create 2x3 plots
        idx = 1
        for row in range(2):
            for col in range(3):
                plot = self.plot_widget.addPlot(row=row, col=col)
                plot.setXRange(-2, 20)
                plot.setYRange(-2, 20)
                plot.setAspectLocked(True)
                plot.showGrid(x=True, y=True)
                plot.setTitle(f"Plot {idx}")
                self.plots[f"plot{idx}"] = plot
                idx += 1

        # Slider
        controls = QHBoxLayout()
        self.slider_label = QLabel("Frame: 0")
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(NUM_FRAMES - 1)
        self.slider.valueChanged.connect(self.on_slider_changed)
        controls.addWidget(self.slider_label)
        controls.addWidget(self.slider)
        main_layout.addLayout(controls)

        self.current_frame = 0
        self.on_slider_changed(0)

    def on_slider_changed(self, value):
        self.current_frame = value
        self.slider_label.setText(f"Frame: {value}")
        self.update_all_plots()

    def update_all_plots(self):
        for name, plot in self.plots.items():
            tracker = self.trackers[name]
            frame_data = self.data_sets[name][self.current_frame]
            tracker.update(frame_data)
            predicted = tracker.get_predictions()
            self.draw_plot(plot, frame_data, predicted)

    def draw_plot(self, plot, detected, predicted):
        plot.clear()

        for cid, data in detected.items():
            pts = data['points']
            if pts.shape[0] > 0:
                scatter = pg.ScatterPlotItem(x=pts[:, 0], y=pts[:, 1], size=6,
                                            pen=None,
                                            brush=pg.mkBrush(100 + cid * 50, 150, 255))
                plot.addItem(scatter)

            cx, cy, _ = data['centroid']
            doppler = data.get('doppler', 0)

            # Check to avoid warnings on empty point sets
            if pts.shape[0] > 0:
                vx = np.mean(pts[:, 0])
                vy = np.mean(pts[:, 1])
                text = f"ID {cid}\nD={doppler:.2f}\nvx={vx:.1f}, vy={vy:.1f}"
            else:
                text = f"ID {cid}\nD={doppler:.2f}\nNo points"

            label = pg.TextItem(text=text, anchor=(0, 0), color='w')
            label.setPos(cx + 0.5, cy + 0.3)
            plot.addItem(label)

        # Draw predicted cluster centroids (those that disappeared)
        for cid, pred in predicted.items():
            px, py, _ = pred
            pred_scatter = pg.ScatterPlotItem([px], [py], symbol='x',
                                            pen=pg.mkPen('r'), size=14)
            plot.addItem(pred_scatter)

            label = pg.TextItem(text=f"Pred {cid}", color='r')
            label.setPos(px + 0.3, py + 0.3)
            plot.addItem(label)


# ------------------------------
# Launch Application
# ------------------------------

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ClusterViewer()
    viewer.show()
    sys.exit(app.exec_())
