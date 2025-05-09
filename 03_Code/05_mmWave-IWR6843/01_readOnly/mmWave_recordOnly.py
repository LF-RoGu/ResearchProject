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
radarSensor_g = DataDecoderTI()
frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
## @}

## @defgroup Thread locks
## @{
IWR6843AoP_list = []
IWR6843AoP_lock = threading.Lock()
## @}

## @defgroup threadFunctions Thread Functions
## @{

def mmWaveSensor_thread():
    """!
    Reads data from the UART from the mmWave sensor, detects frames using a predefined MAGIC WORD,
    and stores valid frames in a thread-safe list for further processing.

    @ingroup threadFunctions
    """
    global IWR6843AoP_list
    global IWR6843AoP_lock

    while True:
        # Polling the IWR6843 sensor and getting the number of decoded frames
        numFrames = radarSensor_g.pollIWR6843()

        # Continuing if there are no new frames
        if numFrames == 0:
            continue

        # Getting the new frames and deleting them from the sensor's internal buffer
        newFrames = radarSensor_g.get_and_delete_decoded_frames(numFrames)

        # Appending the new frames to the global list of decoded frames in a thread-safe way
        with IWR6843AoP_lock:
            IWR6843AoP_list += newFrames


def processing_thread():
    """!
    Continuously retrieves frames, processes them, and prints live information.
    Clears the terminal on every new frame and prints x, y, z, doppler, snr for each point.

    @ingroup threadFunctions
    """
    global IWR6843AoP_lock
    global IWR6843AoP_list

    while True:
        frames_to_process = []

        # Lock and retrieve available frames
        with IWR6843AoP_lock:
            if len(IWR6843AoP_list) == 0:
                continue
            frames_to_process = IWR6843AoP_list
            IWR6843AoP_list = []

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
      - mmWaveSensor_thread
      - processing_thread

    @pre The mmWave sensor must be properly connected and configured.
    @post Sensor data is continuously collected and printed live.
    """
    warnings.filterwarnings('ignore')
    logging.basicConfig(level=LOGGING_LEVEL)

    # Sending the configuration commands to the radar sensor before starting the threads
    radarSensor_g.initIWR6843(SENSOR_CONFIG_PORT_PC, SENSOR_DATA_PORT_PC, SENSOR_CONFIG_FILE)

    # Starting all background threads
    threading.Thread(target=mmWaveSensor_thread, daemon=True).start()
    threading.Thread(target=processing_thread, daemon=True).start()

    # Keep the main thread alive
    while True:
        time.sleep(0)

## @}
## @}  # End of mmWave_readonly group
