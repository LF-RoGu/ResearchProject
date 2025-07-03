#include "SensorData.h"

//#define DEBUG_RAW_FRAME

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

	#ifdef DEBUG_RAW_FRAME
	std::cout << "\n================ RAW FRAME HEX DUMP ================\n";
	size_t count = 0;
	for (uint8_t byte : rawData) {
		printf("%02X ", byte);
		count++;
		if (count % 16 == 0) std::cout << "\n";
	}
	if (count % 16 != 0) std::cout << "\n"; // ensure trailing line

	std::cout << "[DEBUG] Parsed Header Fields:\n";
	std::cout << "Magic Word: 0x" << std::hex << header.getMagicWord() << "\n";
	std::cout << "  Version: " << std::hex << header.getVersion() << "\n";
	std::cout << "  Packet Length: " << std::hex << header.getPacketLength() << "\n";
	std::cout << "  Platform: " << std::hex << header.getPlatform() << "\n";
	std::cout << "  Frame Number: " << std::hex << header.getFrameNumber() << "\n";
	std::cout << "  Timestamp: " << std::hex << header.getTime() << "\n";
	std::cout << "  Num Detected Objects: " << std::hex << header.getNumObjDetecter() << "\n";
	std::cout << "  Num TLVs: " << std::hex << header.getNumTLV() << "\n";
	std::cout << "  Subframe Number: " << std::hex << header.getSubframeNum() << "\n";
	std::cout << "  Remaining bytes in buffer: " << std::hex << rawData.size() << "\n";

	std::cout << "================ END HEX DUMP =====================\n";
	#endif

	//Parsing the data
	header = Frame_header(rawData);

	payload = TLV_payload(rawData, header.getNumObjDetecter(), header.getNumTLV());
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
