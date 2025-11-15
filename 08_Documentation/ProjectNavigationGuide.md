# Radar Odometry Research Project Guide

## 1. Introduction
This repository documents a radar odometry research effort that uses the Texas Instruments IWR6843 mmWave sensor mounted on a Ninebot Go-Kart platform. The system pairs radar data with an MTi-G-710 IMU and an embedded computer (Raspberry Pi 5) to estimate ego-motion reliably in environments where cameras or LiDAR may struggle. Radar provides direct Doppler velocity measurements, giving motion cues that remain robust in adverse lighting and weather. The software stack in this repo captures the entire pipeline: configuring radar chirps, logging raw data, filtering clutter via RANSAC, aggregating submaps, tracking clusters, and validating the trajectory using fused point clouds rendered in Qt tools.

Key goals of the project include:
- Demonstrating radar-only odometry augmented with IMU rotation estimates to reduce hardware cost and complexity.
- Separating static and dynamic objects through Doppler velocity cues, RANSAC filtering, and cluster tracking.
- Building reusable tooling (Python notebooks, C++ services, Qt visualizers) to replay experiments, tune the pipeline, and support future student work.
- Recording and publishing logs, calibration files, and documentation so that new contributors can reproduce experiments end-to-end.

## 2. Repository Structure Overview
Use the following map to understand how assets are organized. The list is ordered roughly in the order you might consult them when onboarding.

### Planning, Models, and References
- `01_Planning/`: Meeting notes, schedules, and early research summaries that capture decisions leading to the current pipeline.
- `02_Models/`: CAD/3D references for the hardware layout (sensor mounts, vehicle integration fixtures).
- `07_Literature/`: Core papers and vendor documentation that motivate radar odometry, Doppler processing, and UART parsing (e.g., "Radar-Only Odometry and Mapping for Autonomous Vehicles").

### Source Code (`03_Code/`)
The main code tree is grouped by implementation language so you can pick the stack that matches your task.

- `01_Python/`: Rapid prototyping scripts and notebooks used during algorithm development.
  - `01_Clustering/`: Experiments with radar point cloud clustering heuristics.
  - `02_Pipeline/`: Early end-to-end pipelines before being ported to C++.
  - `03_VxVyComponents/`: Velocity decomposition utilities for Doppler vectors.
  - `04_MTi-G-710/`: IMU parsing and calibration helpers.
  - `05_mmWave-IWR6843/`: Radar configuration snippets and stand-alone tests.
  - `06_sensorFusion/`: The most mature Python stack, containing:
    - `01_Code-log/`: Scripts used on the vehicle to log synchronized radar/IMU sessions.
    - `02_Code-readLogs/`: Offline readers that parse captured binary logs into analyzable formats.
    - `03_Code-liveViewLogs/`: Live streaming utilities for on-vehicle monitoring.
    - `03_Code-qtViewLogs/`: Python Qt viewers that power the final visualization workflow—open runs such as [`labDriveAroundICP_Full1`](images/labDriveAroundICP_Full1.png) to see the fused radar/IMU output.
    - `test/`: Scratch area for quick experiments against sample data.
  - `07_chirpConfig/`: Radar chirp configuration JSON/TOML files for the IWR6843.
- `02_C++/`: Production-quality services that run on the Raspberry Pi 5 and handle all on-vehicle data acquisition.
  - `01_mmWave-IWR6843/`: Low-level sensor interface for capturing raw radar frames.
  - `02_MTi-G-710/`: IMU acquisition module handling synchronization with radar timestamps.
  - `03_sensorFusion/`: Fusion engine combining radar and IMU measurements into ego-motion estimates.
  - `04_C270HD/`: Logitech C270 HD camera integration for visual context during testing.
- `03_QTFramework/01_pointCloudVisualizer/`: Early Qt experiments kept for reference; use them only when you need to revisit the original prototypes.
- `04_C#/`: Legacy utilities retained for reference when porting older tooling.

### Data and Logs
- `04_Logs/`: Organized test sessions.
  - `01_sensorFusion/`: Multi-sensor captures aligned for fusion validation.
  - `02_IWR6843-standAlone/`: Radar-only recordings for algorithm benchmarking.
  - `03_mti-standAlone/`: IMU-only sessions used to characterize drift and bias.
- `05_test/`: Safe sandbox for temporary experiments; contents can be cleaned without affecting the pipeline.

### Presentations and Documentation
- `06_PresentationUpdates/`: Slide decks (PPTX/ODP) documenting milestone reviews and live demos.
- `08_Documentation/`: LaTeX sources (`main.tex`, section files) and final PDF reports (`RadarOdometry.pdf`); start with `08_Documentation/README.md` for a quick orientation when entering the folder.
  - `images/`: Figures referenced by the report and README (block diagrams, vehicle photos, point-cloud examples).
  - `ProjectNavigationGuide.md` (this document): High-level orientation for new contributors.
- `09_Images/`: Standalone media assets not yet incorporated into the LaTeX report.

## 3. Getting Started Workflow
1. **Study the system design** by reviewing `08_Documentation/section/01introduction.tex` and related sections for context on sensing modalities, vehicle setup, and algorithmic contributions.
2. **Inspect experimental code** in `03_Code/01_Python/06_sensorFusion` to understand logging formats and quick iteration workflows. These scripts are ideal for reproducing plots from the report or trying new filtering ideas.
3. **Deploy embedded services** from `03_Code/02_C++` when you are ready to run on hardware. Each sensor directory contains build files and service entry points targeted at the Raspberry Pi 5.
4. **Replay logs** with the Qt visualizers in `03_Code/03_QTFramework/01_pointCloudVisualizer` or the Python `03_Code-qtViewLogs` utilities. Pair them with datasets from `04_Logs` to verify algorithm changes.
5. **Update documentation** by editing the LaTeX sources in `08_Documentation/section/` or by extending this guide as the repository evolves.

## 4. Objective for Future Students
This repository is intentionally comprehensive so new researchers can:
- Trace every decision from planning to final validation.
- Re-use sensor configuration files to bring up the IWR6843 and MTi-G-710 quickly.
- Compare radar-only odometry against fused pipelines using the shared log sets.
- Build on the provided visualization and presentation assets when communicating results.

If you add new sensors, data sources, or processing steps, mirror the existing structure (planning → code → logs → documentation) so that successors can follow the same breadcrumbs. Feel free to append new subsections here describing additional folders or workflows as the project grows.
