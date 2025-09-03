import sys
import pandas as pd
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel
)
from PyQt5.QtCore import Qt
import pyqtgraph as pg

class RadarVisualizer(QWidget):
    def __init__(self, csv_path_1, csv_path_2):
        super().__init__()
        self.setWindowTitle("Radar Point Visualizer (2D) - Dual Dataset")
        self.resize(1200, 600)

        # Load both datasets
        self.data1 = pd.read_csv(csv_path_1)
        self.data2 = pd.read_csv(csv_path_2)

        # Determine frame ranges
        self.frames1 = sorted(self.data1['frame_id'].unique())
        self.frames2 = sorted(self.data2['frame_id'].unique())
        self.max_frame1 = max(self.frames1)
        self.max_frame2 = max(self.frames2)

        # Setup UI
        layout = QHBoxLayout()
        self.setLayout(layout)

        self.panel1 = self.create_panel("Dataset 1", self.max_frame1)
        self.panel2 = self.create_panel("Dataset 2", self.max_frame2)

        layout.addLayout(self.panel1["layout"])
        layout.addLayout(self.panel2["layout"])

        # Initial plot
        self.update_plot(self.panel1, self.data1, 1)
        self.update_plot(self.panel2, self.data2, 1)

    def create_panel(self, title, max_frame):
        panel = {}
        panel["layout"] = QVBoxLayout()

        # Plot
        panel["plot_widget"] = pg.PlotWidget(title=title)
        panel["plot"] = panel["plot_widget"].plot([], [], pen=None, symbol='o', symbolBrush='b')
        panel["plot_widget"].setLabel('left', 'Y')
        panel["plot_widget"].setLabel('bottom', 'X')
        panel["plot_widget"].setAspectLocked(True)

        # Slider
        panel["slider"] = QSlider(Qt.Horizontal)
        panel["slider"].setMinimum(1)
        panel["slider"].setMaximum(max_frame)
        panel["slider"].setValue(1)
        panel["label"] = QLabel(f"Frame ID: 1")

        # Connect with update function
        panel["slider"].valueChanged.connect(
            lambda val, p=panel, d=title: self.slider_callback(p, val, d)
        )

        # Assemble layout
        panel["layout"].addWidget(panel["plot_widget"])
        panel["layout"].addWidget(panel["slider"])
        panel["layout"].addWidget(panel["label"])

        return panel

    def slider_callback(self, panel, value, dataset_id):
        if dataset_id == "Dataset 1":
            self.update_plot(panel, self.data1, value)
        elif dataset_id == "Dataset 2":
            self.update_plot(panel, self.data2, value)

    def update_plot(self, panel, dataset, frame_id):
        subset = dataset[dataset["frame_id"] == frame_id]
        x = subset["x"].values
        y = subset["y"].values
        panel["plot"].setData(x, y)
        panel["label"].setText(f"Frame ID: {frame_id}")

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Manually set CSV file paths
    csv_path_1 = "radar_SensorA.csv"
    csv_path_2 = "radar_SensorB.csv"

    viewer = RadarVisualizer(csv_path_1, csv_path_2)
    viewer.show()
    sys.exit(app.exec_())
