# 02_Models

This directory stores the CAD assets that describe every physical mounting component used in the radar odometry platform. Open
these assemblies to understand how sensors were packaged on the Ninebot go-kart or to export geometry for manufacturing.

## Contents

| Folder/File | Description |
| --- | --- |
| `01_CylindricalMountPin/`, `02_HexMountPin/` | Parametric pins that lock the sensor plates to the chassis. Use them when 3D-printing replacements. |
| `03_IWR6843AOP/` | Source models for the TI IWR6843 radar enclosure, including mounting hole patterns. |
| `04_Microcontroller_placeholder/` | Placeholder footprint that documents the control electronics dimensions when routing harnesses. |
| `05_MTi-G-710/` | MTi-G-710 IMU bracket and anti-vibration mount geometry. |
| `06_NineBot/` | Base vehicle frame, used as the reference for aligning every other component. |
| `07_wireHolder/` | Cable guides that keep radar, IMU, and power wiring secured. |
| `NineBot.SLDPRT`, `NineBot 3D View_Left_28March.SLDPRT` | Stand-alone reference views of the vehicle body. |
| `SensorAssmbly.SLDASM` | Top-level assembly showing how all mounts combine around the vehicle. |

When updating or adding models, keep the folder numbering consistent so downstream scripts that load assemblies continue to work.
