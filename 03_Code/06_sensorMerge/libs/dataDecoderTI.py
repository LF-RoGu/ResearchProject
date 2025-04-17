import os
import serial
import time
import numpy as np
import threading

import libs.parser_mmw_demo as pmd

__location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

class DataDecoderTI:
    def __init__(self, maximumDataBuffer=4096, maximumNumDecodedFrames=4096):
        self.maximumNumDecodedFrames = maximumNumDecodedFrames
        self.maximumDataBuffer = maximumDataBuffer
        self.configPort = {}
        self.dataPort = {}
        self.dataBuffer = np.array([], dtype='uint8')
        self.decodedFrames = []
        self.lock = threading.Lock()

    def initIWR6843(self, configPort, dataPort, configFile, configPortBaudRate=115200, dataPortBaudRate=921600):
        #Setting up the serial port
        self.configPort = serial.Serial(configPort, configPortBaudRate)
        self.dataPort = serial.Serial(dataPort, dataPortBaudRate)

        #Reading in the configuration file and sending it line by line
        configFileContent = [line.rstrip('\r\n') for line in open(os.path.join(__location__, configFile))]
        for i in configFileContent:
            self.configPort.write((i+'\n').encode())
            print(i)
            time.sleep(0.01)

    def pollIWR6843(self):
        readBuffer = self.dataPort.read(self.dataPort.in_waiting)
        
        if len(readBuffer) == 0:
            return 0
        
        newData = np.frombuffer(readBuffer, dtype = 'uint8')
        self.dataBuffer = np.append(self.dataBuffer, newData)
        
        while len(self.dataBuffer) > self.maximumDataBuffer:
            self.dataBuffer = np.delete(self.dataBuffer, 1, 0)

        try:
            startIndex, _, _, _, _ = pmd.parser_helper(self.dataBuffer, len(self.dataBuffer), False)
        except:
            return 0

        if startIndex == -1:
            return 0

        newFrames = []
        while True:
            try:
                parser_result, \
                headerStartIndex,  \
                totalPacketNumBytes, \
                numDetObj,  \
                numTlv,  \
                subFrameNumber,  \
                detectedX_array,  \
                detectedY_array,  \
                detectedZ_array,  \
                detectedV_array,  \
                detectedRange_array,  \
                detectedAzimuth_array,  \
                detectedElevation_array,  \
                detectedSNR_array,  \
                detectedNoise_array = pmd.parser_one_mmw_demo_output_packet(self.dataBuffer, len(self.dataBuffer), False)

                if parser_result != 0:
                    break

                decodedFrame = {}
                decodedFrame["subFrameNumber"] = subFrameNumber
                decodedFrame["detectedPoints"] = []
                for i in range(numDetObj):
                    point = {}
                    point["x"] = detectedX_array[i]
                    point["y"] = detectedY_array[i]
                    point["z"] = detectedZ_array[i]
                    point["doppler"] = detectedV_array[i]
                    point["snr"] = detectedSNR_array[i]
                    point["noise"] = detectedNoise_array[i]
                    
                    decodedFrame["detectedPoints"].append(point)

                newFrames.append(decodedFrame)

                self.dataBuffer = self.dataBuffer[headerStartIndex+totalPacketNumBytes::1]
            except:
                print("An error occured while trying to parse the data")
                break

        with self.lock:
            self.decodedFrames = self.decodedFrames + newFrames
            while len(self.decodedFrames) > self.maximumNumDecodedFrames:
                self.decodedFrames.pop(0)

        return len(newFrames)

    def closeIWR6843(self):
        self.configPort.write(('sensorStop\n').encode())
        self.configPort.close()
        self.dataPort.close()

    def get_and_delete_decoded_frames(self, n):
        with self.lock:
            n = min(n, len(self.decodedFrames))
            frames_to_return = self.decodedFrames[:n]
            self.decodedFrames = self.decodedFrames[n:]
        return frames_to_return