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
import csv

# Local Imports
from dataDecoderTI import IWR6843AoP
from frameAggregator import FrameAggregator
from mtiDecoder import MTi_G_710

## @defgroup Global Constants
## @{
PLATFORM_EMBEDDED = False
LOGGING_LEVEL = logging.DEBUG

SENSOR_CONFIG_FILE = "profile_azim60_elev30_optimized.cfg"
SENSOR_CONFIG_PORT_PC = "COM6"
SENSOR_DATA_PORT_PC = "COM5"

FRAME_AGGREGATOR_NUM_PAST_FRAMES = 0
IWR6843AoP_FPS = 30

LOGGING_SUFIX = "_driveAround_Fun"
## @}

## @defgroup Pipeline Constructors
## @{
radarSensor_g = IWR6843AoP()
imuSensor_g = MTi_G_710()

frame_aggregator = FrameAggregator(FRAME_AGGREGATOR_NUM_PAST_FRAMES)
## @}

## @defgroup Thread locks
## @{   
IWR6843AoP_list = []
IWR6843AoP_lock = threading.Lock()
IWR6843AoP_ready = threading.Event()
MTi_G_710_list = []
MTi_G_710_lock = threading.Lock()
MTi_G_710_ready = threading.Event()
Sensor_read_lock = threading.Lock()
Sensor_read_ready = threading.Event()
## @}


## @defgroup threadFunctions Thread Functions
## @{

def IWR6843AoP_thread():
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
            IWR6843AoP_list.append(newFrames)
        # Set a lock that the processing was done, and the information is ready to go
        IWR6843AoP_ready.set()

def MTi_G_710_thread():
    """!
    @brief Continuously reads packets from the IMU and stores the latest one.

    @ingroup threadFunctions
    """
    global MTi_G_710_list
    global MTi_G_710_lock

    while True:
        MTi_data = imuSensor_g.read_loop(4)
        if MTi_data:
            with MTi_G_710_lock:
                MTi_G_710_list.append(MTi_data)
            MTi_G_710_ready.set()

def sensor_read_thread():
    """!
    Reads data from the UART from the mmWave sensor, detects frames using a predefined MAGIC WORD,
    and stores valid frames in a thread-safe list for further processing.

    @ingroup threadFunctions
    """
    global IWR6843AoP_list
    global MTi_G_710_list
    global Sensor_read_lock

    while True:
        # Polling the IWR6843 sensor and getting the number of decoded frames
        numFrames = radarSensor_g.pollIWR6843()
        
        # Continuing if there are no new frames
        if numFrames == 0:
            continue

        # Getting the new frames and deleting them from the sensor's internal buffer
        newFrames = radarSensor_g.get_and_delete_decoded_frames(numFrames)
        #
        MTi_data = imuSensor_g.read_loop(numFrames)

        with Sensor_read_lock:
            # Appending the new frames to the global list of decoded frames in a thread-safe way
            IWR6843AoP_list.append(newFrames)
            MTi_G_710_list.append(MTi_data)
        Sensor_read_ready.set()

def processing_thread():
    """!
    Continuously retrieves frames, processes them, and prints live information.
    Clears the terminal on every new frame and prints x, y, z, doppler, snr for each point.

    @ingroup threadFunctions
    """
    global IWR6843AoP_list
    global MTi_G_710_list
    global Sensor_read_lock

    while True:
        # Wait for both sensors to have new data
        Sensor_read_ready.wait()
        
        # Lock and retrieve available frames
        with Sensor_read_lock:
            IWR6843AoP_data = list(IWR6843AoP_list)
            IWR6843AoP_list.clear()
            IWR6843AoP_ready.clear()
            MTi_G_710_data = list(MTi_G_710_list)
            MTi_G_710_list.clear()
            MTi_G_710_ready.clear()

        try:
            os.system('cls' if os.name == 'nt' else 'clear')  # Clear terminal for live effect
            # Processing each frame
            for frame in IWR6843AoP_data:   
                print(f"\n--- Processing Frame {frame.get('subFrameNumber', 'N/A')} ---", flush=True)               
                # Print mmWave point-cloud data
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
                if MTi_G_710_data:
                        latest_imu = MTi_G_710_data[-1]  # Get the most recent IMU packet
                        print("\n--- IMU Data ---", flush=True)

                        if "acceleration" in latest_imu:
                            acc = latest_imu["acceleration"]
                            print(f"Acceleration -> x: {acc['x']:.2f}, y: {acc['y']:.2f}, z: {acc['z']:.2f}", flush=True)

                        if "gyroscope" in latest_imu:
                            gyr = latest_imu["gyroscope"]
                            print(f"Gyroscope    -> x: {gyr['x']:.2f}, y: {gyr['y']:.2f}, z: {gyr['z']:.2f}", flush=True)

                        if "magnetometer" in latest_imu:
                            mag = latest_imu["magnetometer"]
                            print(f"Magnetometer -> x: {mag['x']:.2f}, y: {mag['y']:.2f}, z: {mag['z']:.2f}", flush=True)

                        if "quaternion" in latest_imu:
                            quat = latest_imu["quaternion"]
                            print(f"Quaternion   -> q0: {quat['q0']:.2f}, q1: {quat['q1']:.2f}, q2: {quat['q2']:.2f}, q3: {quat['q3']:.2f}", flush=True)

                        if "euler" in latest_imu:
                            eul = latest_imu["euler"]
                            print(f"Euler angles -> Roll: {eul['roll']:.2f}, Pitch: {eul['pitch']:.2f}, Yaw: {eul['yaw']:.2f}", flush=True)

                        if "position" in latest_imu:
                            pos = latest_imu["position"]
                            print(f"Position     -> Latitude: {pos['latitude']:.6f}, Longitude: {pos['longitude']:.6f}", flush=True)

                        if "altitude" in latest_imu:
                            alt = latest_imu["altitude"]
                            print(f"Altitude     -> {alt:.2f} meters", flush=True)

                        if "velocity" in latest_imu:
                            vel = latest_imu["velocity"]
                            print(f"Velocity     -> East: {vel['east']:.2f}, North: {vel['north']:.2f}, Up: {vel['up']:.2f}", flush=True)


        except Exception as e:
            logging.error(f"Error decoding frame: {e}")

def storage_thread():
    global IWR6843AoP_list
    global MTi_G_710_list
    global Sensor_read_lock
    global Sensor_read_ready
    global imu_writer, radar_writer, imu_csv_file, radar_csv_file

    while True:
        # 1) wait until both a radar batch and an IMU packet are available
        Sensor_read_ready.wait()

        
        with Sensor_read_lock:
            # 2) pull exactly one radar batch
            radar_batches = IWR6843AoP_list.pop(0)
            # 3) pull exactly one IMU frame
            imu_frame = MTi_G_710_list.pop(0)
            if not IWR6843AoP_list or not MTi_G_710_list:
                Sensor_read_ready.clear()

        # 4) for each frame in that radar batch, assign a shared frame_id and write both IMU and radar
        for frame in radar_batches:
            fid = radarSensor_g.radar_frame_counter
            radarSensor_g.radar_frame_counter += 1

            for imu_data in imu_frame:
                imu_data["frame_id"] = fid  # shared frame_id for all subframes

                imu_writer.writerow([
                    imu_data.get("frame_id", -1),
                    imu_data.get("subframe_id", -1),
                    imu_data.get("acceleration", {}).get("x", ""),
                    imu_data.get("acceleration", {}).get("y", ""),
                    imu_data.get("acceleration", {}).get("z", ""),
                    imu_data.get("gyroscope", {}).get("x", ""),
                    imu_data.get("gyroscope", {}).get("y", ""),
                    imu_data.get("gyroscope", {}).get("z", ""),
                    imu_data.get("magnetometer", {}).get("x", ""),
                    imu_data.get("magnetometer", {}).get("y", ""),
                    imu_data.get("magnetometer", {}).get("z", ""),
                    imu_data.get("euler", {}).get("roll", ""),
                    imu_data.get("euler", {}).get("pitch", ""),
                    imu_data.get("euler", {}).get("yaw", ""),
                    imu_data.get("position", {}).get("latitude", ""),
                    imu_data.get("position", {}).get("longitude", ""),
                    imu_data.get("altitude", ""),
                    imu_data.get("velocity", {}).get("east", ""),
                    imu_data.get("velocity", {}).get("north", ""),
                    imu_data.get("velocity", {}).get("up", "")
                ])


            # write radar rows
            points = frame.get("detectedPoints", [])
            for idx, pt in enumerate(points):
                radar_writer.writerow([
                    fid,
                    idx + 1,
                    pt.get("x", ""),
                    pt.get("y", ""),
                    pt.get("z", ""),
                    pt.get("doppler", ""),
                    pt.get("snr", ""),
                    pt.get("noise", "")
                ])

        # 5) flush to disk
        imu_csv_file.flush()
        radar_csv_file.flush()

## @}

# Main program entry point
if __name__ == "__main__":
    """!
    Initializes and starts sensor and processing threads.

    @section Threads Started:
      - IWR6843AoP_thread
      - processing_thread

    @pre The mmWave sensor must be properly connected and configured.
    @post Sensor data is continuously collected and printed live.
    """
    warnings.filterwarnings('ignore')
    logging.basicConfig(level=LOGGING_LEVEL)

    # Open CSV files and write headers
    imu_csv_file = open(f'imu_data{LOGGING_SUFIX}.csv', mode='w', newline='')
    imu_writer = csv.writer(imu_csv_file)
    imu_writer.writerow([
        "frame_id", "subframe_id", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",
        "mag_x", "mag_y", "mag_z", "roll", "pitch", "yaw"
    ])

    radar_csv_file = open(f'radar_data{LOGGING_SUFIX}.csv', mode='w', newline='')
    radar_writer = csv.writer(radar_csv_file)
    radar_writer.writerow([
        "frame_id", "point_id", "x", "y", "z", "doppler", "snr", "noise"
    ])



    # Sending the configuration commands to the radar sensor before starting the threads
    radarSensor_g.initIWR6843(SENSOR_CONFIG_PORT_PC, SENSOR_DATA_PORT_PC, SENSOR_CONFIG_FILE)
    imuSensor_g.initialize()

    # Sleep to give the sensor time to calibrate
    time.sleep(1.0)

    # Starting all background threads
    threading.Thread(target=sensor_read_thread, daemon=True).start()
    #threading.Thread(target=processing_thread, daemon=True).start()
    threading.Thread(target=storage_thread, daemon=True).start() 

    # Keep the main thread alive
    while True:
        time.sleep(0)

## @}
## @}  # End of mmWave_readonly group
