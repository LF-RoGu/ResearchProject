#include "SensorData.h"

//#define DEBUG_RAW_FRAME
#define DEBUG_FRAME_HEADER
#define DEBUG_FRAME_TLV

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
	std::cout << "================ END HEX DUMP =====================\n";
	#endif
	#ifdef DEBUG_FRAME_HEADER
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

	#ifdef DEBUG_FRAME_TLV
	std::cout << "\n================== SENSOR DATA DEBUG ==================\n";

	// Detected Points
	for (size_t j = 0; j < payload_data.DetectedPoints_str.size(); ++j) {
		const DetectedPoints detectedPoint = payload_data.DetectedPoints_str[j];
		std::cout << "Converted Floats:\n";
		std::cout << "  x: " << detectedPoint.x_f << " meters\n";
		std::cout << "  y: " << detectedPoint.y_f << " meters\n";
		std::cout << "  z: " << detectedPoint.z_f << " meters\n";
		std::cout << "  doppler: " << detectedPoint.doppler_f << " m/s\n";
	}

	// Side Info Points
	for (size_t j = 0; j < payload_data.SideInfoPoint_str.size(); ++j) {
		const SideInfoPoint sideInfo = payload_data.SideInfoPoint_str[j];
		std::cout << "[DEBUG] SideInfo #" << j
				<< " SNR: " << sideInfo.snr
				<< " Noise: " << sideInfo.noise << "\n";
	}

	// Range Profile Peaks
	for (size_t j = 0; j < payload_data.RangeProfilePoint_str.size(); ++j) {
		const RangeProfilePoint peak = payload_data.RangeProfilePoint_str[j];
		std::cout << "[PEAK] Bin #" << peak.bin_u16
				<< " | Range: " << peak.range_f << " m"
				<< " | Power: " << peak.power_u16 << "\n";
	}

	std::cout << "=======================================================\n";
	#endif
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
