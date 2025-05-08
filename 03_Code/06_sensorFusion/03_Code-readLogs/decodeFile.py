import os
import csv
from dataclasses import dataclass
from fileSearch import find_project_root

@dataclass
class RadarRecord:
    frame_id: int
    subframe: int
    x: float
    y: float
    z: float
    doppler: float
    snr: float
    noise: float
@dataclass
class ImuRecord:
    frame_id: int
    subframe: int
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
        "subframe",
        "x", "y", "z",
        "doppler", 
        "snr", 
        "noise"
    ]

    def __init__(self, file_name: str = "radar_data_30_04_2025.csv", folder_name: str = "01_Logs-30042025",csv_path: str = None):
        """
        If csv_path is provided, it will be used directly.
        Otherwise, the path will be resolved from the file_name and project structure.
        """
        if csv_path:
            self.csv_path = csv_path
        else:
            script_dir   = os.path.dirname(os.path.abspath(__file__))
            project_root = find_project_root(script_dir, "ResearchProject")
            self.csv_path = os.path.join(
                project_root,
                "04_Logs", "01_sensorFusion", folder_name,
                file_name
            )

    def _row_to_record(self, row: dict) -> RadarRecord:
        # convert the strings in row to the correct types
        return RadarRecord(
            frame_id = int(row["frame_id"]),
            subframe = int(row["subframe"]),
            x        = float(row["x"]),
            y        = float(row["y"]),
            z        = float(row["z"]),
            doppler  = float(row["doppler"]),
            snr      = float(row["snr"]) / 10,
            noise    = float(row["noise"]) / 10
        )

    def load_all(self) -> list[RadarRecord]:
        """Read the entire CSV into a list of RadarRecord."""
        records = []
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f, fieldnames=self.FIELDNAMES)
            next(reader)  # skip header
            for row in reader:
                records.append(self._row_to_record(row))
        return records

    def __iter__(self):
        """Make the loader itself iterable."""
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f, fieldnames=self.FIELDNAMES)
            next(reader)
            for row in reader:
                yield self._row_to_record(row)

class ImuCSVReader:
    FIELDNAMES = [
        "frame_id", 
        "subframe",
        "acc_x", "acc_y", "acc_z",
        "gyro_x", "gyro_y", "gyro_z",
        "mag_x", "mag_y", "mag_z",
        "roll", "pitch", "yaw"
    ]

    def __init__(self, file_name: str = "imu_data_30_04_2025.csv", folder_name: str = "01_Logs-30042025" ,csv_path: str = None):
        """
        If csv_path is provided, it will be used directly.
        Otherwise, the path will be resolved from the file_name and project structure.
        """
        if csv_path:
            self.csv_path = csv_path
        else:
            script_dir   = os.path.dirname(os.path.abspath(__file__))
            project_root = find_project_root(script_dir, "ResearchProject")
            self.csv_path = os.path.join(
                project_root,
                "04_Logs", "01_sensorFusion", folder_name ,
                file_name
            )
    def _row_to_record(self, row: dict) -> ImuRecord:
        # convert the strings in row to the correct types
        return ImuRecord(
            frame_id = int(row["frame_id"]),
            subframe = int(row["subframe"]),
            acc_x    = float(row["acc_x"]),
            acc_y    = float(row["acc_y"]),
            acc_z    = float(row["acc_z"]),
            gyro_x   = float(row["gyro_x"]),
            gyro_y   = float(row["gyro_y"]),
            gyro_z   = float(row["gyro_z"]),
            mag_x    = float(row["mag_x"]),
            mag_y    = float(row["mag_y"]),
            mag_z    = float(row["mag_z"]),
            roll     = float(row["roll"]),
            pitch    = float(row["pitch"]),
            yaw      = float(row["yaw"])
        )

    def load_all(self) -> list[ImuRecord]:
        """Read the entire CSV into a list of RadarRecord."""
        records = []
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f, fieldnames=self.FIELDNAMES)
            next(reader)  # skip header
            for row in reader:
                records.append(self._row_to_record(row))
        return records

    def __iter__(self):
        """Make the loader itself iterable."""
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f, fieldnames=self.FIELDNAMES)
            next(reader)
            for row in reader:
                yield self._row_to_record(row)
# --------------------
# Example usage:
# --------------------
if __name__ == "__main__":
    radarLoader = RadarCSVReader(file_name="radar_data_30_04_2025.csv")
    imuLoader = ImuCSVReader(file_name="imu_data_30_04_2025.csv")

    imuRecords = imuLoader.load_all()
    for imu_rec in imuRecords:
        print(f"{imu_rec.frame_id}.{imu_rec.subframe}")
        print(f"→ Acceleration (X,Y,Z): ({imu_rec.acc_x:.3f}, {imu_rec.acc_y:.3f}, {imu_rec.acc_z:.3f})")
        print(f"→ Gyroscope (X,Y,Z): ({imu_rec.gyro_x:.3f}, {imu_rec.gyro_y:.3f}, {imu_rec.gyro_z:.3f})")
        print(f"→ Magnometer (X,Y,Z): ({imu_rec.mag_x:.3f}, {imu_rec.mag_y:.3f}, {imu_rec.mag_z:.3f})")
        print(f"→ Motion (Yaw, Pitch, Roll): ({imu_rec.roll:.3f}, {imu_rec.pitch:.3f}, {imu_rec.roll:.3f})")
    
    all_records = radarLoader.load_all()
    for radar_rec in all_records:
        print(f"{radar_rec.frame_id}.{radar_rec.subframe}")
        print(f"→ ({radar_rec.x:.3f}, {radar_rec.y:.3f}, {radar_rec.z:.3f})")
        print(f"→ Doppler={radar_rec.doppler:.2f}, SNR={radar_rec.snr:.2f}, Noise={radar_rec.noise:.2f}")    
    