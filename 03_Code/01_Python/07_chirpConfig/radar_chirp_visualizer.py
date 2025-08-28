import sys
import re
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QFileDialog, QLabel
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import numpy as np
from pathlib import Path

class ChirpVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TI Radar Chirp Visualizer")
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

    
        self.plot_widget = PlotWidget(title="Chirp Frequency vs Time")
        self.plot_widget.setLabel('left', 'Frequency (GHz)')
        self.plot_widget.setLabel('bottom', 'Time (µs)')
        self.plot_widget.addLegend()
        self.already_labeled_files = set()
        self.layout.addWidget(self.plot_widget)

        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)

        self.label = QLabel("Load .cfg files to visualize chirps")
        self.layout.addWidget(self.label)

        self.button = QPushButton("Add Config File")
        self.button.clicked.connect(self.load_config)
        self.layout.addWidget(self.button)

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
        total_chirps = chirps_per_loop * loops

        time_axis = []
        freq_axis = []
        annotations = []

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

            t = np.linspace(0, ramp_end_time, 100)
            f = start_freq + (slope * t) / 1000  # convert MHz/us to GHz

            for i in range(loops):
                t_shifted = t + i * (idle_time + ramp_end_time)
                time_axis.append(t_shifted)
                freq_axis.append(f)
                annotations.append(f"{label_prefix} | Chirp {chirp_index} | Loop {i}")

        color = self.colors[self.config_count % len(self.colors)]
        for tx, fx in zip(time_axis, freq_axis):
            self.plot_widget.plot(tx, fx, pen=pg.mkPen(color, width=1.5))

        # Add a single legend entry per file
        if label_prefix not in self.already_labeled_files:
            self.plot_widget.plot([], [], pen=pg.mkPen(color, width=1.5), name=label_prefix)
            self.already_labeled_files.add(label_prefix)


        self.config_count += 1

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ChirpVisualizer()
    window.resize(1000, 600)
    window.show()
    sys.exit(app.exec_())
