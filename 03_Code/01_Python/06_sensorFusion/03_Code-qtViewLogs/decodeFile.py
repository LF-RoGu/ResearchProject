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
    imu_idx: int

    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float

    accel_x: float
    accel_y: float
    accel_z: float

    free_accel_x: float
    free_accel_y: float
    free_accel_z: float

    delta_v_x: float
    delta_v_y: float
    delta_v_z: float

    delta_q_w: float
    delta_q_x: float
    delta_q_y: float
    delta_q_z: float

    gyro_x: float   # maps to CSV 'rate_x'
    gyro_y: float   # maps to CSV 'rate_y'
    gyro_z: float   # maps to CSV 'rate_z'

    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float

    mag_x: float
    mag_y: float
    mag_z: float

    temperature: float

    status_byte: int
    packet_counter: int
    time_fine: float

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

    def load_all(self, filter_enabled: bool = False, start_frame: int = 0) -> list[RadarRecord]:
        grouped_frames = defaultdict(list)
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                record = self._row_to_record(row)
                if record:
                    if not filter_enabled or record.frame_id > start_frame:
                        grouped_frames[record.frame_id].append(record)
        return [grouped_frames[frame_id] for frame_id in sorted(grouped_frames)]

    
class ImuCSVReader:
    FIELDNAMES = [
        "frame_id", "imu_idx",
        "accel_x", "accel_y", "accel_z",
        "free_accel_x", "free_accel_y", "free_accel_z",
        "delta_v_x", "delta_v_y", "delta_v_z",
        "delta_q_w", "delta_q_x", "delta_q_y", "delta_q_z",
        "rate_x", "rate_y", "rate_z",
        "quat_w", "quat_x", "quat_y", "quat_z",
        "mag_x", "mag_y", "mag_z",
        "temperature",
        "status_byte",
        "packet_counter",
        "time_fine"
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
                frame_id       = int(row["frame_id"]),
                imu_idx        = int(row["imu_idx"]),

                # raw accel
                accel_x          = float(row["accel_x"]),
                accel_y          = float(row["accel_y"]),
                accel_z          = float(row["accel_z"]),
                free_accel_x     = float(row["free_accel_x"]),
                free_accel_y     = float(row["free_accel_y"]),
                free_accel_z     = float(row["free_accel_z"]),

                # velocity increments (integrated accel)
                delta_v_x      = float(row["delta_v_x"]),
                delta_v_y      = float(row["delta_v_y"]),
                delta_v_z      = float(row["delta_v_z"]),

                # delta orientation quaternion
                delta_q_w      = float(row["delta_q_w"]),
                delta_q_x      = float(row["delta_q_x"]),
                delta_q_y      = float(row["delta_q_y"]),
                delta_q_z      = float(row["delta_q_z"]),

                # gyro rates
                gyro_x         = float(row["rate_x"]),
                gyro_y         = float(row["rate_y"]),
                gyro_z         = float(row["rate_z"]),

                # absolute orientation quaternion
                quat_w         = float(row["quat_w"]),
                quat_x         = float(row["quat_x"]),
                quat_y         = float(row["quat_y"]),
                quat_z         = float(row["quat_z"]),

                # magnetometer
                mag_x          = float(row["mag_x"]),
                mag_y          = float(row["mag_y"]),
                mag_z          = float(row["mag_z"]),

                # temperature
                temperature    = float(row["temperature"]),

                # status
                status_byte    = int(row["status_byte"]),

                # package counter
                packet_counter = int(row["packet_counter"]),

                # timestamp
                time_fine      = float(row["time_fine"]),
            )
        except (ValueError, KeyError) as e:
            print(f"⚠️ Skipping malformed IMU row: {row} — {e}")
            return None

    def load_all(self, filter_enabled: bool = False, start_frame: int = 0) -> list[ImuRecord]:
        grouped_frames = defaultdict(list)
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                record = self._row_to_record(row)
                if record:
                    if not filter_enabled or record.frame_id > start_frame:
                        grouped_frames[record.frame_id].append(record)
        return [grouped_frames[frame_id] for frame_id in sorted(grouped_frames)]
