from libs.dataDecoderTI import DataDecoderTI
import time
import os

# Replace these with the actual serial ports on your system
CONFIG_PORT = 'COM6'  # Enhanced port for config
DATA_PORT = 'COM5'    # Standard port for data
CONFIG_FILE = os.path.join("configs", "profile_azim60_elev30_optimized.cfg")

def main():
    decoder = DataDecoderTI()

    try:
        print("Initializing sensor...")
        decoder.initIWR6843(CONFIG_PORT, DATA_PORT, CONFIG_FILE)

        print("\nReading data frames. Press Ctrl+C to stop.\n")
        while True:
            num_frames = decoder.pollIWR6843()

            if num_frames > 0:
                frames = decoder.get_and_delete_decoded_frames(num_frames)
                for frame in frames:
                    print(f"--- Frame {frame['subFrameNumber']} ---")
                    for i, point in enumerate(frame["detectedPoints"]):
                        print(f"Point {i+1}: x={point['x']:.2f}, y={point['y']:.2f}, z={point['z']:.2f}, "
                              f"doppler={point['doppler']:.2f}, snr={point['snr']:.2f}")
                    print()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping sensor...")
    finally:
        decoder.closeIWR6843()
        print("Sensor closed.")

if __name__ == "__main__":
    main()
