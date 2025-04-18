"""""! 
    @file PipelineLive.py
    @brief Pipeline for real-time object detection and collision prevention using mmWave sensor.
    @details This script processes radar sensor data to detect static obstacles, estimate velocity,
    and trigger emergency braking when necessary.

    @defgroup mmWave_readonly Pipeline Live
    @brief Real-time processing pipeline.
    @{
"""

# Imports
import time
import threading
import warnings
import logging
import numpy as np
import os

# Local Imports
from dataDecoderTI import DataDecoderTI
from frameAggregator import FrameAggregator

## @defgroup Global Constants
## @{
PLATFORM_EMBEDDED = False
LOGGING_LEVEL = logging.DEBUG

SENSOR_CONFIG_FILE = "profile_azim60_elev30_optimized.cfg"
SENSOR_CONFIG_PORT_PC = "COM6"
SENSOR_DATA_PORT_PC = "COM5"

FRAME_AGGREGATOR_NUM_PAST_FRAMES = 0
## @}

## @defgroup Pipeline Constructors
## @{
radarSensor = DataDecoderTI()
frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
## @}

## @defgroup Thread locks
## @{
frame_list = []
frame_lock = threading.Lock()
processed_data_lock = threading.Lock()
## @}

## @defgroup Global variables
## @{
latest_self_speed_filtered = 0
latest_dbscan_clusters = []
## @}

## @defgroup threadFunctions Thread Functions
## @{

def sensor_thread():
    """!
    Reads data from the UART from the mmWave sensor, detects frames using a predefined MAGIC WORD,
    and stores valid frames in a thread-safe list for further processing.

    @ingroup threadFunctions
    """
    global radarSensor
    global frame_list
    global frame_lock

    while True:
        # Polling the IWR6843 sensor and getting the number of decoded frames
        numFrames = radarSensor.pollIWR6843()

        # Continuing if there are no new frames
        if numFrames == 0:
            continue

        # Getting the new frames and deleting them from the sensor's internal buffer
        newFrames = radarSensor.get_and_delete_decoded_frames(numFrames)

        # Appending the new frames to the global list of decoded frames in a thread-safe way
        with frame_lock:
            frame_list += newFrames


def processing_thread():
    """!
    Continuously retrieves frames, processes them, and prints live information.
    Clears the terminal on every new frame and prints x, y, z, doppler, snr for each point.

    @ingroup threadFunctions
    """
    global frame_lock
    global frame_list

    while True:
        frames_to_process = []

        # Lock and retrieve available frames
        with frame_lock:
            if len(frame_list) == 0:
                continue
            frames_to_process = frame_list
            frame_list = []

        try:
            # Processing each frame
            for frame in frames_to_process:
                os.system('cls' if os.name == 'nt' else 'clear')  # Clear terminal for live effect
                print(f"\n--- Processing Frame {frame.get('subFrameNumber', 'N/A')} ---", flush=True)

                for idx, point in enumerate(frame.get("detectedPoints", [])):
                    print(
                        f"Point {idx + 1}: "
                        f"x={point['x']:.2f}, "
                        f"y={point['y']:.2f}, "
                        f"z={point['z']:.2f}, "
                        f"doppler={point['doppler']:.2f}, "
                        f"snr={point['snr']:.2f}",
                        flush=True
                    )

                # Optional: store the frame if needed
                frame_aggregator.updateBuffer(frame)

        except Exception as e:
            logging.error(f"Error decoding frame: {e}")

## @}

# Main program entry point
if __name__ == "__main__":
    """!
    Initializes and starts sensor and processing threads.

    @section Threads Started:
      - sensor_thread
      - processing_thread

    @pre The mmWave sensor must be properly connected and configured.
    @post Sensor data is continuously collected and printed live.
    """
    warnings.filterwarnings('ignore')
    logging.basicConfig(level=LOGGING_LEVEL)

    # Sending the configuration commands to the radar sensor before starting the threads
    radarSensor.initIWR6843(SENSOR_CONFIG_PORT_PC, SENSOR_DATA_PORT_PC, SENSOR_CONFIG_FILE)

    # Starting all background threads
    threading.Thread(target=sensor_thread, daemon=True).start()
    threading.Thread(target=processing_thread, daemon=True).start()

    # Keep the main thread alive
    while True:
        time.sleep(0)

## @}
## @}  # End of mmWave_readonly group
