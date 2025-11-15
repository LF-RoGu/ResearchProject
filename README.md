# ResearchProject

This repository documents the complete development pipeline—from early planning and CAD work to embedded software, visualizations, presentations, and the final report. The structure is designed to help new contributors quickly understand where each component lives and how the system fits together.

## Repository Structure

- [01_Planning](01_Planning/): Project planning artefacts, schedules, and research that guided the implementation roadmap.

- [02_Models](02_Models/): 3D models and CAD resources created for the hardware layout (summarized in [02_Models/README.md](02_Models/README.md)).

- [03_Code](03_Code/): Source code for experiments, embedded services, and visualization tools.
  - [01_Python](03_Code/01_Python/): Prototype scripts and data-processing experiments used during development—the final visualization dashboards also live here.
    - [06_sensorFusion](03_Code/01_Python/06_sensorFusion/): Focused notebooks and scripts for validating sensor fusion logic prior to deployment.
      - [03_Code-qtViewLogs](03_Code/01_Python/06_sensorFusion/03_Code-qtViewLogs/): Final Qt-based views for replaying and inspecting recorded sensor logs (see the `labDriveAroundICP_Full1` visualization).
      - [03_Code-liveViewLogs](03_Code/01_Python/06_sensorFusion/03_Code-liveViewLogs/): Utilities for streaming fused data live during test sessions.
  - [02_C++](03_Code/02_C++/): Production modules compiled for the Raspberry Pi 5, organized by sensor subsystem; these services handle the on-vehicle data acquisition that feeds the shared logs.
    - [01_mmWave-IWR6843](03_Code/02_C++/01_mmWave-IWR6843/): Interfaces with the mmWave radar to capture and stream radar data.
    - [02_MTi-G-710](03_Code/02_C++/02_MTi-G-710/): Handles IMU data acquisition and synchronization.
    - [03_sensorFusion](03_Code/02_C++/03_sensorFusion/): Combines individual sensor feeds into a unified view used in the live system.
    - [04_C270HD](03_Code/02_C++/04_C270HD/): Manages the Logitech C270 HD camera for visual context.
  - [03_QTFramework](03_Code/03_QTFramework/): Early Qt experiments retained for reference (they are not part of the final toolchain).
    - [01_pointCloudVisualizer](03_Code/03_QTFramework/01_pointCloudVisualizer/): Prototype GUI for log visualization; compare it against the polished Python `03_Code-qtViewLogs` utilities above.
  - [04_C#](03_Code/04_C#/): Legacy utilities retained for reference.

- [04_Logs](04_Logs/): Raw recordings and datasets captured during testing and validation (see the dedicated [folder README](04_Logs/README.md) for naming conventions).
  - [01_sensorFusion](04_Logs/01_sensorFusion/): Multi-sensor captures aligned for fusion analysis.
  - [02_IWR6843-standAlone](04_Logs/02_IWR6843-standAlone/): Radar-only sessions for algorithm calibration.
  - [03_mti-standAlone](04_Logs/03_mti-standAlone/): IMU-focused recordings for sensor characterization.

- [05_test](05_test/): Sandbox area for temporary experiments and files that are safe to remove.

- [06_PresentationUpdates](06_PresentationUpdates/): Iterative presentation decks documenting project progress and milestones.

- [07_Literature](07_Literature/): Research papers, notes, and references that informed the system design.

- [08_Documentation](08_Documentation/): Final report and supplementary documentation describing the full project (start with the concise [folder README](08_Documentation/README.md)).
  - [ProjectNavigationGuide.md](08_Documentation/ProjectNavigationGuide.md): A narrative overview of the project goals, sensing hardware, and directory structure to help new contributors ramp up quickly.
  - [images](08_Documentation/images/): Reference figures illustrating the hardware layout, calibration process, and data products described in the report (see previews below).

## Visual Highlights

The images below (stored in `09_Images/`) provide a quick overview of the system. You can replace or update them at any time; see [`09_Images/README.md`](09_Images/README.md) for details.

<div align="center">

![Block diagram summarizing how each sensor integrates with the Raspberry Pi 5](09_Images/ProjectSummaryIntegration.png)  
*System integration overview showing how the radars, IMU, and camera connect to the Raspberry Pi pipeline.*

![Annotated photo calling out every sensor installed on the go-kart](09_Images/ProjectSensorHighlights.png)  
*Annotated platform view identifying all mounted sensors.*

<img src="09_Images/3DModelFullSensor.PNG" width="360"> <img src="09_Images/MTi3DModel.png" width="360">  
*CAD references for the radar/IMU enclosure and IMU mounting bracket.*

![Pipeline overview describing the C++ processing stages](09_Images/CppImplementationOverview.png)  
*High-level view of the processing flow from raw sensor frames to fusion, encoding, visualization, and logging.*

</div>

## How to Navigate the Code

1. Review Python prototypes in `03_Code/01_Python`, especially the `06_sensorFusion` folder, to understand how sensor data was processed during early experimentation.  
2. Explore the Raspberry Pi services in `03_Code/02_C++` to see how each sensor’s data enters the production fusion pipeline.  
3. Use the Qt visualization tools in `03_Code/01_Python/06_sensorFusion/03_Code-qtViewLogs` when inspecting recorded logs. For comparison, the early Qt experiments are archived in `03_Code/03_QTFramework`.  
4. Reference `06_PresentationUpdates` and `08_Documentation` for broader explanations, system context, and presentation materials.
