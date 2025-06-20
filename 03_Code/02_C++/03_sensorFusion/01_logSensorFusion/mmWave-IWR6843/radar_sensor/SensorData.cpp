#include "SensorData.h"


SensorData::SensorData()
{
}

SensorData::SensorData(vector<uint8_t> rawData)
{
	//Setting the timestamp
	timestamp = chrono::system_clock::now();

	//Copying the raw data
	storedRawData.reserve(rawData.size());
	copy(rawData.begin(), rawData.begin() + rawData.size(), back_inserter(storedRawData));

	//Parsing the data
	header = Frame_header(rawData);
	payload = TLV_payload(rawData, header.getNumObjDetecter());
	payload_data = payload.getTLVFramePayloadData();
}

Frame_header SensorData::getHeader()
{
	return header;
}

TLV_payload SensorData::getTLVPayload()
{
	return payload;
}

TLVPayloadData SensorData::getTLVPayloadData()
{
	return payload_data;
}
