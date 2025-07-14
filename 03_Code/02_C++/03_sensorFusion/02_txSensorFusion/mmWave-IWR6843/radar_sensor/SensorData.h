#ifndef SENSORDATA_H
#define SENSORDATA_H

#pragma once
#include <vector>
#include <stdint.h>
#include <chrono>
#include <ctime>
#include "sensor_data/UARTFrame.h"

using namespace std;

class SensorData
{
private:
	Frame_header header;
	TLV_payload payload;
	std::vector<TLVPayloadData> payload_data_vect;

public:
	SensorData();
	SensorData(vector<uint8_t> rawData);
	
	Frame_header getHeader() const;
	TLV_payload getTLVPayload() const;
	std::vector<TLVPayloadData> getTLVPayloadData() const;

	//Vector and timestamp
	chrono::time_point<chrono::system_clock> timestamp;
	vector<uint8_t> storedRawData;

};

#endif // !SENSORDATA_H