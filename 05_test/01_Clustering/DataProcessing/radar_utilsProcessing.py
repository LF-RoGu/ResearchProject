import struct
import math
import pandas as pd
# Parse Frame Header
def parse_frame_header(raw_data_list):
    if len(raw_data_list) < 40:
        raise ValueError("Insufficient data for Frame Header")
    raw_bytes = bytes([raw_data_list.pop(0) for _ in range(40)])
    frame_header = struct.unpack('<QIIIIIIII', raw_bytes)
    return {
        "Magic Word": f"0x{frame_header[0]:016X}",
        "Version": f"0x{frame_header[1]:08X}",
        "Total Packet Length": frame_header[2],
        "Platform": f"0x{frame_header[3]:08X}",
        "Frame Number": frame_header[4],
        "Time [in CPU Cycles]": frame_header[5],
        "Num Detected Obj": frame_header[6],
        "Num TLVs": frame_header[7],
        "Subframe Number": frame_header[8]
    }

# Parse TLV Header
def parse_tlv_header(raw_data_list):
    if len(raw_data_list) < 8:
        raise ValueError("Insufficient data for TLV Header")
    raw_bytes = bytes([raw_data_list.pop(0) for _ in range(8)])
    tlv_type, tlv_length = struct.unpack('<II', raw_bytes)
    return {"TLV Type": tlv_type, "TLV Length": tlv_length}

# Parse Type 1: Detected Points
def parse_type_1_data(tlv_header, raw_data_list):
    """Parses Type 1 TLV payload (Detected Points)."""
    payload_length = tlv_header["TLV Length"]
    point_size = 16  # Each point has 16 bytes: X, Y, Z, Doppler
    num_points = payload_length // point_size

    detected_points = []
    for _ in range(num_points):
        if len(raw_data_list) < point_size:
            #print("Warning: Insufficient data for Type 1 point.")
            break
        point_bytes = bytes([raw_data_list.pop(0) for _ in range(point_size)])
        x, y, z, doppler = struct.unpack('<ffff', point_bytes)

        # Calculate range profile from x, y, z
        comp_detected_range = math.sqrt((x * x) + (y * y) + (z * z))

        # Calculate azimuth from x, y
        if y == 0:
            detected_azimuth = 90 if x >= 0 else -90
        else:
            detected_azimuth = math.atan(x / y) * (180 / math.pi)

        # Calculate elevation angle from x, y, z
        if x == 0 and y == 0:
            detected_elev_angle = 90 if z >= 0 else -90
        else:
            detected_elev_angle = math.atan(z / math.sqrt((x * x) + (y * y))) * (180 / math.pi)

        # Append to detected points with additional info
        detected_points.append({
            "X [m]": x,
            "Y [m]": y,
            "Z [m]": z,
            "Doppler [m/s]": doppler,
            "Range [m]": comp_detected_range,
            "Azimuth [deg]": detected_azimuth,
            "Elevation Angle [deg]": detected_elev_angle
        })

    return {"Type 1 Data": detected_points}


# Parse Type 2: Placeholder for additional payloads
def parse_type_2_data(tlv_header, raw_data_list):
    payload_length = tlv_header["TLV Length"]
    payload = raw_data_list[:payload_length]
    raw_data_list[:payload_length] = []
    return {"Type 2 Data": payload}

# Parse Type 3: Another placeholder for raw data
def parse_type_3_data(tlv_header, raw_data_list):
    payload_length = tlv_header["TLV Length"]
    payload = raw_data_list[:payload_length]
    raw_data_list[:payload_length] = []
    return {"Type 3 Data": payload}

# Parse Type 7: Side Info (SNR and Noise)
def parse_type_7_data(tlv_header, raw_data_list, num_detected_obj):
    payload_length = tlv_header["TLV Length"]
    expected_length = 4 * num_detected_obj  # 4 bytes per point (2 SNR, 2 Noise)

    if payload_length != expected_length:
        #print(f"Warning: Type 7 length mismatch. Expected {expected_length}, got {payload_length}.")
        raw_data_list[:payload_length] = []
        return {"Side Info": []}

    side_info = []
    for _ in range(num_detected_obj):
        if len(raw_data_list) < 4:
            #print("Warning: Insufficient data for Type 7 point.")
            break
        point_bytes = bytes([raw_data_list.pop(0) for _ in range(4)])
        snr, noise = struct.unpack('<HH', point_bytes)
        side_info.append({"SNR [dB]": snr * 0.1, "Noise [dB]": noise * 0.1})

    return {"Side Info": side_info}

# Process the log file with optional SNR and Z[m] filtering
def process_log_file(file_path, snr_threshold=None, z_min=None, z_max=None, doppler_threshold=None):
    frames_dict = {}
    data = pd.read_csv(file_path, names=["Timestamp", "RawData"], skiprows=1)

    for row_idx in range(len(data)):
        try:
            if pd.isnull(data.iloc[row_idx]['RawData']):
                print(f"Skipping row {row_idx + 1}: Null data.")
                continue

            raw_data_list = [int(x) for x in data.iloc[row_idx]['RawData'].split(',')]
            frame_header = parse_frame_header(raw_data_list)
            frame_number = frame_header["Frame Number"]

            detected_points = []
            side_info = []

            for _ in range(frame_header["Num TLVs"]):
                tlv_header = parse_tlv_header(raw_data_list)
                tlv_type = tlv_header["TLV Type"]

                # Parse Type 1 - Detected Points
                if tlv_type == 1:
                    detected_points = parse_type_1_data(tlv_header, raw_data_list)["Type 1 Data"]
                # Parse Type 7 - Side Info
                elif tlv_type == 7:
                    num_detected_obj = frame_header["Num Detected Obj"]
                    side_info = parse_type_7_data(tlv_header, raw_data_list, num_detected_obj)["Side Info"]
                # Placeholder for other types
                elif tlv_type in [2, 3]:
                    parse_type_2_data(tlv_header, raw_data_list)
                else:
                    print(f"Unknown TLV Type {tlv_type} in Frame {frame_number}. Skipping.")
                    raw_data_list[:tlv_header["TLV Length"]] = []

            # Apply Filtering (SNR, Z[m], Doppler)
            if detected_points and side_info:
                filtered_points = []
                filtered_info = []

                for i in range(len(detected_points)):
                    snr = side_info[i]["SNR [dB]"]
                    z_val = detected_points[i]["Z [m]"]
                    doppler = detected_points[i]["Doppler [m/s]"]

                    # Filter conditions (only if thresholds are provided)
                    snr_valid = snr_threshold is None or snr >= snr_threshold
                    z_valid = ((z_min is None or z_val >= z_min) and
                               (z_max is None or z_val <= z_max))
                    doppler_valid = doppler_threshold is None or abs(doppler) >= doppler_threshold

                    if snr_valid and z_valid and doppler_valid:
                        filtered_points.append(detected_points[i])
                        filtered_info.append(side_info[i])

                # Save filtered data only if non-empty
                if filtered_points:
                    frames_dict[frame_number] = {"Frame Header": frame_header, "TLVs": []}
                    frames_dict[frame_number]["TLVs"].append({"Type 1 Data": filtered_points})
                    frames_dict[frame_number]["TLVs"].append({"Side Info": filtered_info})

        except (ValueError, IndexError) as e:
            print(f"Error parsing row {row_idx + 1}: {e}")

    return frames_dict