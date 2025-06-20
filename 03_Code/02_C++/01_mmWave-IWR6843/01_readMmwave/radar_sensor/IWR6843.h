#ifndef IWR6843_H
#define IWR6843_H

#pragma once
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <stdint.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <pthread.h>

#include "SensorData.h"

using namespace std;

class IWR6843
{
private:
	int configPort_fd;
	int dataPort_fd;

	vector<uint8_t> dataBuffer;
	vector<SensorData> decodedFrameBuffer;
	pthread_mutex_t decodedFrameBufferMutex;

	int configSerialPort(int port_fd, int baudRate);
	int sendConfigFile(int fd, const std::string& filePath);
	vector<size_t> findIndexesOfMagicWord();
	vector<vector<uint8_t>> splitIntoSublistsByIndexes(const vector<size_t>& indexes);

public:
	IWR6843();
	~IWR6843();
	int init(const std::string& configPort,
             const std::string& dataPort,
             const std::string& configFilePath);
	int poll();
	bool copyDecodedFramesFromTop(vector<SensorData>& destination, uint numFrames, bool delFrames, long timeout_ms);
	bool getDecodedFramesSize(int& destination, long timeout_ms);
};

#endif // !IWR6843_H