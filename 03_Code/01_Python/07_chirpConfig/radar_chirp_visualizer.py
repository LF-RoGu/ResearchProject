import sys
import re
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QFileDialog, QLabel
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import pyqtgraph.exporters
import numpy as np
from pathlib import Path


class ChirpVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TI Radar Chirp Visualizer")
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        # Custom left-aligned title using QLabel
        title_label = QLabel("Chirp Frequency vs Time")
        title_label.setStyleSheet("font-size: 18pt; color: black;")
        title_label.setAlignment(pg.Qt.QtCore.Qt.AlignLeft)  # force left alignment
        # Add left margin (in pixels)
        title_label.setContentsMargins(100, 0, 0, 0)  # (left, top, right, bottom)
        self.layout.addWidget(title_label)

        # Create plot widget (no built-in title)
        self.plot_widget = PlotWidget()
        self.layout.addWidget(self.plot_widget)

        # Larger axis labels with offset to avoid collisions
        self.plot_widget.getAxis('left').setLabel(
            text='Frequency (GHz)', 
            **{'color': 'black', 'font-size': '14pt'}, 
            offset=50
        )
        self.plot_widget.getAxis('bottom').setLabel(
            text='Time (µs)', 
            **{'color': 'black', 'font-size': '14pt'}, 
            offset=35
        )

        # Larger tick numbers
        font = pg.Qt.QtGui.QFont()
        font.setPointSize(14)
        self.plot_widget.getAxis('left').setTickFont(font)
        self.plot_widget.getAxis('bottom').setTickFont(font)

        self.plot_widget.addLegend()
        self.already_labeled_files = set()

        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)

        self.label = QLabel("Load .cfg files to visualize chirps")
        self.layout.addWidget(self.label)

        self.button = QPushButton("Add Config File")
        self.button.clicked.connect(self.load_config)
        self.layout.addWidget(self.button)

        # Button to save graph as high-res PNG
        save_btn = QPushButton("Save Graph as PNG")
        save_btn.clicked.connect(self.save_graph)
        self.layout.addWidget(save_btn)

        self.colors = ['r', 'g', 'b', 'c', 'm', 'y', 'w']
        self.config_count = 0

    def load_config(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open .cfg file", "", "Config Files (*.cfg *.txt)")
        if not file_path:
            return

        with open(file_path, 'r') as f:
            cfg_lines = f.readlines()

        profile_cfgs = []
        chirp_cfgs = []
        frame_cfg = None

        for line in cfg_lines:
            if line.startswith("profileCfg"):
                profile_cfgs.append([float(x) for x in line.strip().split()[1:]])
            elif line.startswith("chirpCfg"):
                chirp_cfgs.append([int(x) for x in line.strip().split()[1:]])
            elif line.startswith("frameCfg"):
                frame_cfg = [float(x) for x in line.strip().split()[1:]]

        if not profile_cfgs or not chirp_cfgs or not frame_cfg:
            self.label.setText("Incomplete config in file: " + Path(file_path).name)
            return

        self.plot_chirps(profile_cfgs, chirp_cfgs, frame_cfg, Path(file_path).stem)

    def plot_chirps(self, profiles, chirps, frame, label_prefix):
        chirps_per_loop = int(frame[1])
        loops = int(frame[2])

        time_axis = []
        freq_axis = []

        for chirp_index in range(chirps_per_loop):
            chirp = chirps[chirp_index]
            profile_id = chirp[2]
            if profile_id >= len(profiles):
                continue  # skip if chirp references undefined profile
            profile = profiles[profile_id]

            start_freq = profile[1]  # GHz
            idle_time = profile[2]   # µs
            ramp_end_time = profile[4]  # µs
            slope = profile[7]      # MHz/us

            t = np.linspace(0, ramp_end_time, 50)
            f = start_freq + (slope * t) / 1000  # convert MHz/us to GHz

            for i in range(loops):
                t_shifted = t + i * (idle_time + ramp_end_time)
                time_axis.append(t_shifted)
                freq_axis.append(f)

        color = self.colors[self.config_count % len(self.colors)]
        for tx, fx in zip(time_axis, freq_axis):
            self.plot_widget.plot(tx, fx, pen=pg.mkPen(color, width=3))  # thicker lines

        # Add a single legend entry per file
        if label_prefix not in self.already_labeled_files:
            self.plot_widget.plot([], [], pen=pg.mkPen(color, width=3), name=label_prefix)
            self.already_labeled_files.add(label_prefix)

        self.config_count += 1

    def save_graph(self):
        exporter = pg.exporters.ImageExporter(self.plot_widget.plotItem)
        exporter.parameters()['width'] = 2000  # high resolution
        exporter.export("chirp_plot.png")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ChirpVisualizer()
    window.resize(1000, 600)
    window.show()
    sys.exit(app.exec_())
