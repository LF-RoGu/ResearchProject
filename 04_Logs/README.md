# 04_Logs

The logs in this directory are the authoritative source for replaying experiments, validating new algorithms, and matching the
plots published in the documentation. Each subfolder groups captures by acquisition mode so you can quickly locate the data that
matches your testing scenario.

## Folder map

| Folder | Description |
| --- | --- |
| `01_sensorFusion/` | Full radar + IMU sessions. Filenames follow the pattern `<sensor>_<scenario>.csv` (for example `radarA_labDriveAround3.csv`). Pair the CSVs with the synchronized videos inside the nested `video/` folders when recreating demos. |
| `02_IWR6843-standAlone/` | Radar-only recordings used for chirp tuning and clutter rejection experiments. |
| `03_mti-standAlone/` | IMU-only captures for bias estimation and drift analysis. |

### Usage tips

1. Sync the log timestamps with the Python `03_Code-qtViewLogs` dashboards to inspect runs such as [`labDriveAroundICP_Full1`](../08_Documentation/images/labDriveAroundICP_Full1.png).
2. When generating new datasets, copy one of the existing folder structures so downstream scripts can parse them automatically.
3. Archive raw exports (CSV/AVI) alongside derived products so others can reproduce your plots.
