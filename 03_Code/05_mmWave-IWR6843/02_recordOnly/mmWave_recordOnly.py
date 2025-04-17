"""!
    @file PipelineLive.py
    @brief Pipeline for real-time object detection and collision prevention using mmWave sensor.
    @details This script processes radar sensor data to detect static obstacles, estimate velocity,
    and trigger emergency braking when necessary.

    @defgroup Pipeline_V2 Pipeline Live
    @brief Real-time processing pipeline.
    @{
"""

# Imports
import time
import logging
import os
import numpy as np
from dataDecoderTI import DataDecoderTI

## @defgroup Global Constants
## @{
PLATFORM_EMBEDDED = False
LOGGING_LEVEL = logging.DEBUG

## @brief Setting the sensor's config file filename
SENSOR_CONFIG_FILE = "profile_azim60_elev30_optimized.cfg"
## @brief UART port used for sensor configuration on the embedded linux device.
SENSOR_CONFIG_PORT_EMBEDDED = "/dev/ttyUSB0"
## @brief UART port used for receiving sensor data on the embedded linux device.
SENSOR_DATA_PORT_EMBEDDED = "/dev/ttyUSB1"
## @brief UART port used for sensor configuration on the Windows PC.
SENSOR_CONFIG_PORT_PC = "COM6"
## @brief UART port used for receiving sensor data Windows PC.
SENSOR_DATA_PORT_PC = "COM5"
## @}

if __name__ == "__main__":
    """!
    @brief Main program entry point.

    This script initializes and runs a live processing loop to monitor mmWave radar data in real-time,
    printing frame information to the terminal using flush and line overwrite for efficient output.

    @pre The mmWave sensor must be properly connected and configured.
    @post Sensor data is continuously collected and printed to the terminal.
    """
    logging.basicConfig(level=LOGGING_LEVEL)

    print("Initializing radar sensor...")
    radarSensor = DataDecoderTI()

    # Sending the configuration commands to the radar sensor before entering main loop
    if PLATFORM_EMBEDDED:
        radarSensor.initIWR6843(SENSOR_CONFIG_PORT_EMBEDDED, SENSOR_DATA_PORT_EMBEDDED, SENSOR_CONFIG_FILE)
    else:
        radarSensor.initIWR6843(SENSOR_CONFIG_PORT_PC, SENSOR_DATA_PORT_PC, SENSOR_CONFIG_FILE)

    print("Started. Press Ctrl+C to stop.\n")

    try:
        while True:
            # Polling the IWR6843 sensor and getting the number of decoded frames
            num_frames = radarSensor.pollIWR6843()
            if num_frames > 0:
                # Getting and processing each new frame
                frames = radarSensor.get_and_delete_decoded_frames(num_frames)

                for frame in frames:
                    s = f"Frame {frame.get('subFrameNumber', 'N/A')} | "
                    for idx, point in enumerate(frame.get("detectedPoints", [])):
                        s += (
                            f"[{idx+1}: "
                            f"x={point['x']:.2f}, y={point['y']:.2f}, z={point['z']:.2f}, "
                            f"doppler={point['doppler']:.2f}, snr={point['snr']:.2f}] "
                        )
                    print(f"{s}\r", end="", flush=True)  # Print on same line using carriage return

            time.sleep(0.01)  # Small sleep to prevent busy waiting

    except KeyboardInterrupt:
        print("\nStopped by user.")
        radarSensor.closeIWR6843()
        print("Sensor closed.")

## @}  # End of Pipeline_V2 group
