import os
import csv
from dataclasses import dataclass
from fileSearch import find_project_root
from collections import defaultdict

@dataclass
class RadarRecord:
    frame_id: int
    point_id: int
    x: float
    y: float
    z: float
    doppler: float
    snr: float
    noise: float
@dataclass
class ImuRecord:
    frame_id: int
    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: float
    mag_y: float
    mag_z: float
    roll: float
    pitch: float
    yaw: float

class RadarCSVReader:
    FIELDNAMES = [
        "frame_id", 
        "point_id",
        "x", "y", "z",
        "doppler", 
        "snr", 
        "noise"
    ]

    def __init__(self, file_name: str = "radar_data_30_04_2025.csv", folder_name: str = "01_Logs-30042025", csv_path: str = None):
        if csv_path:
            self.csv_path = csv_path
        else:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = find_project_root(script_dir, "ResearchProject")
            self.csv_path = os.path.join(project_root, "04_Logs", "01_sensorFusion", folder_name, file_name)

    def _row_to_record(self, row: dict) -> RadarRecord | None:
        try:
            return RadarRecord(
                frame_id=int(row["frame_id"]),
                point_id=int(row["point_id"]),
                x=float(row["x"]),
                y=float(row["y"]),
                z=float(row["z"]),
                doppler=float(row["doppler"]),
                snr=float(row["snr"]) / 10,
                noise=float(row["noise"]) / 10
            )
        except (ValueError, KeyError) as e:
            print(f"⚠️ Skipping malformed radar row: {row} — {e}")
            return None

    def load_all(self) -> list[RadarRecord]:
        grouped_frames = defaultdict(list)
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                record = self._row_to_record(row)
                if record:
                    grouped_frames[record.frame_id].append(record)
        return [grouped_frames[frame_id] for frame_id in sorted(grouped_frames)]
    
class ImuCSVReader:
    FIELDNAMES = [
        "frame_id",
        "accel_x", "accel_y", "accel_z",
        "rate_x", "rate_y", "rate_z",
        "mag_x", "mag_y", "mag_z",
        "roll", "pitch", "yaw" 
    ]


    def __init__(self, file_name: str = "imu_data_30_04_2025.csv", folder_name: str = "01_Logs-30042025", csv_path: str = None):
        if csv_path:
            self.csv_path = csv_path
        else:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = find_project_root(script_dir, "ResearchProject")
            self.csv_path = os.path.join(project_root, "04_Logs", "01_sensorFusion", folder_name, file_name)

    def _row_to_record(self, row: dict) -> ImuRecord | None:
        try:
            return ImuRecord(
                frame_id=int(row["frame_id"]),
                acc_x=float(row["accel_x"]),
                acc_y=float(row["accel_y"]),
                acc_z=float(row["accel_z"]),
                gyro_x=float(row["rate_x"]),
                gyro_y=float(row["rate_y"]),
                gyro_z=float(row["rate_z"]),
                mag_x=float(row["mag_x"]),
                mag_y=float(row["mag_y"]),
                mag_z=float(row["mag_z"]),
                roll=0.0, pitch=0.0, yaw=0.0  # Placeholder
            )
        except (ValueError, KeyError) as e:
            print(f"⚠️ Skipping malformed IMU row: {row} — {e}")
            return None

    def load_all(self) -> list[ImuRecord]:
        grouped_frames = defaultdict(list)
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                record = self._row_to_record(row)
                if record:
                    grouped_frames[record.frame_id].append(record)
        return [grouped_frames[frame_id] for frame_id in sorted(grouped_frames)]