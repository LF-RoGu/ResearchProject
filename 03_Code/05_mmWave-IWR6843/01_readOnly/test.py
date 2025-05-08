import os
import pandas as pd

def decode(file_path):
        csvData = pd.read_csv(file_path)
        decodedFrames = dataToFrames(csvData)

def dataToFrames(data):
    #Preparing a return list containing all frames
    decodedFrames = []
    
    for i in range(len(data)):
        try:
            #Checking if the row is valid (non-null)
            if pd.isnull(data.iloc[i]['RawData']):
                continue

            #Geting the raw data from the current row
            raw_data_list = [int(x) for x in data.iloc[i]['RawData'].split(',')]

            #Parsing the frame header
            frame_header = parse_frame_header(raw_data_list)
            num_tlvs = frame_header["Num TLVs"]

            decodedFrame = None
            decodedSNRandNoise = None

            # Parse TLVs
            for _ in range(num_tlvs):
                tlv_header = parse_tlv_header(raw_data_list)
                if tlv_header["TLV Type"] == 1:  # Interested in Detected Points
                    tlv_payload = parse_tlv_payload(tlv_header, raw_data_list)
                    if tlv_payload and "detectedPoints" in tlv_payload:
                        decodedFrame = {"detectedPoints":tlv_payload["detectedPoints"]}
                elif tlv_header["TLV Type"] == 7: # Interested in Side Info of Detected Points
                    tlv_payload = parse_tlv_payload(tlv_header, raw_data_list)
                    if tlv_payload and "SNRandNoise" in tlv_payload:
                        decodedSNRandNoise = tlv_payload["SNRandNoise"]
            
            #Adding SNR and noise info to the decoded frames
            for point in range(len(decodedFrame["detectedPoints"])):
                decodedFrame["detectedPoints"][point]["snr"] = decodedSNRandNoise[point]["snr"]
                decodedFrame["detectedPoints"][point]["noise"] = decodedSNRandNoise[point]["noise"]

            #Adding the decoded frame to the list of decoded frames
            decodedFrames.append(decodedFrame)

        except Exception as e:
            print(f"Error processing row {i + 1}: {e}")

    return decodedFrames