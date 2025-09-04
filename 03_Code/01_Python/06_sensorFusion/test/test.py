import socket
import time
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

TCP_IP = "192.168.38.97"
TCP_PORT = 8888
RETRY_INTERVAL = 1.0  # Seconds


class RadarViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Radar Viewer (Sensor A & B)")

        # === Layout with 2 PlotWidgets ===
        layout = QHBoxLayout()
        self.plotA = pg.PlotWidget(title="Sensor A - Point Cloud")
        self.plotB = pg.PlotWidget(title="Sensor B - Point Cloud")

        for plot in [self.plotA, self.plotB]:
            plot.setXRange(-10, 10)
            plot.setYRange(0, 15)
            plot.setLabel("left", "Y (forward) [m]")
            plot.setLabel("bottom", "X (sideways) [m]")
            plot.showGrid(x=True, y=True)

        layout.addWidget(self.plotA)
        layout.addWidget(self.plotB)
        self.setLayout(layout)

        # === Scatter plot handles ===
        self.scatterA = pg.ScatterPlotItem(pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 120), size=7)
        self.scatterB = pg.ScatterPlotItem(pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 120), size=7)
        self.plotA.addItem(self.scatterA)
        self.plotB.addItem(self.scatterB)

        # === TCP connection ===
        self.sock = self.wait_for_connection()
        self.buffer = ""
        self.radarA_points = []
        self.radarB_points = []

        # === Timer for reading and updating plots ===
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_data)
        self.timer.start(30)  # ms

    def wait_for_connection(self):
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((TCP_IP, TCP_PORT))
                sock.setblocking(False)  # ⚠️ Important line
                print(f"[PYTHON] Connected to {TCP_IP}:{TCP_PORT}")
                return sock
            except socket.error:
                print(f"[PYTHON] Server not ready, retrying in {RETRY_INTERVAL} sec...")
                time.sleep(RETRY_INTERVAL)

    def read_data(self):
        try:
            data = self.sock.recv(4096)
            if data:
                self.buffer += data.decode('utf-8')
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    self.parse_line(line.strip())
        except BlockingIOError:
            pass  # No data available yet
        except Exception as e:
            print(f"[PYTHON ERROR] {e}")
            self.timer.stop()

    def parse_line(self, line):
        if line.startswith("RADAR_A,"):
            self.radarA_points.clear()
            self.expectedA = int(line.split(',')[1])
            self.readingA = True
            self.readingB = False
            return
        if line.startswith("RADAR_B,"):
            self.radarB_points.clear()
            self.expectedB = int(line.split(',')[1])
            self.readingA = False
            self.readingB = True
            return
        if line.startswith("IMU,"):
            # Optionally print IMU data or process later
            return

        parts = line.split(',')
        if len(parts) != 5:
            return  # Invalid line

        try:
            _, x, y, z, doppler = map(float, parts)
            if self.readingA:
                self.radarA_points.append({'pos': (x, y)})
                if len(self.radarA_points) == self.expectedA:
                    self.scatterA.setData(self.radarA_points)
            elif self.readingB:
                self.radarB_points.append({'pos': (x, y)})
                if len(self.radarB_points) == self.expectedB:
                    self.scatterB.setData(self.radarB_points)
        except ValueError:
            return


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = RadarViewer()
    viewer.resize(1000, 500)
    viewer.show()
    sys.exit(app.exec_())
