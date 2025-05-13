# Pipeline

This subsection contains multiple scripts to process radar sensor data stored in CSV files. The scripts facilitate data extraction, filtering, and visualization to differentiate between noise and valuable data.

## Main Scripts

### Pipeline.py
Processes recorded radar data logs, applying various filtering techniques and visualizing the results. Key features:
- Reads radar logs and parses frames.
- Filters data based on SNR, Z-axis, and Doppler thresholds.
- Uses clustering to identify relevant object groups.
- Generates real-time plots for data analysis.

### PipelineLive.py
Handles live radar data acquisition and processing, enabling real-time visualization and filtering. Key features:
- Connects to the radar sensor and receives data via UART.
- Applies the same processing pipeline as Pipeline.py but in real-time.
- Uses Kalman filtering for self-speed estimation.
- Displays live occupancy grid and cluster visualizations.

### PipelineLiveRaspBerri.py
A variant of PipelineLive.py optimized for Raspberry Pi, incorporating GPIO control for additional functionalities. Key features:
- Implements brake control via GPIO.
- Adjusts processing for embedded systems.
- Maintains real-time data acquisition and filtering.

## Utility Modules

### brakeController.py
Controls a braking system based on detected object proximity. Key features:
- Activates brakes when an obstacle is within a defined range.
- Releases brakes when safe or after a cooldown period.

### dataDecoder.py
Decodes raw radar data into structured information. Key features:
- Parses frame headers and TLVs.
- Extracts detected points, range profiles, and other radar outputs.
- Converts data into a usable format for further processing.

### dbCluster.py
Clusters detected radar points using the DBSCAN algorithm. Key features:
- Identifies object groups based on point cloud data.
- Computes cluster properties like centroid, priority, and velocity.

### frameAggregator.py
Aggregates multiple radar frames for enhanced data consistency. Key features:
- Maintains a buffer of past frames.
- Returns combined point clouds for processing.

### kalmanFilter.py
Implements a Kalman filter for self-speed estimation. Key features:
- Smooths velocity estimates over time.
- Reduces measurement noise for more accurate motion tracking.

### occupancyGrid.py
Constructs an occupancy grid for environment mapping. Key features:
- Transforms point cloud data into a 2D occupancy map.
- Supports Cartesian and polar grid representations.

### pointFilter.py
Filters radar points based on various criteria. Key features:
- Filters by SNR, Cartesian coordinates, and spherical coordinates.
- Implements Doppler-based velocity filtering.

### selfSpeedEstimator.py
Estimates vehicle velocity from radar data. Key features:
- Uses polynomial fitting to calculate self-speed.
- Extracts Doppler velocity components from detected points.

### veSpeedFilter.py
Processes velocity estimates and filters outliers. Key features:
- Calculates velocity estimates from Doppler measurements.
- Filters speed values using an interquartile range (IQR) method.



---
This repository provides a complete processing pipeline for radar-based object detection, environment mapping, and real-time tracking. Modify and extend the scripts as needed to fit specific applications.
