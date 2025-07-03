#include "SensorData.h"

//#define DEBUG_RAW_FRAME
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
	payload_data_vect = payload.getTLVPayloadData();

	#ifdef DEBUG_FRAME_TLV
	std::cout << "\n================== SENSOR DATA DEBUG ==================\n";
	for (size_t i = 0; i < payload_data_vect.size(); ++i) {
	const TLVPayloadData& tlv = payload_data_vect[i];

	for (size_t j = 0; j < tlv.DetectedPoints_str.size(); ++j) {
		const DetectedPoints& dp = tlv.DetectedPoints_str[j];
		std::cout << "Converted Floats:\n";
		std::cout << "  x: " << dp.x_f << " meters\n";
		std::cout << "  y: " << dp.y_f << " meters\n";
		std::cout << "  z: " << dp.z_f << " meters\n";
		std::cout << "  doppler: " << dp.doppler_f << " m/s\n";
	}

	for (size_t j = 0; j < tlv.SideInfoPoint_str.size(); ++j) {
		const SideInfoPoint& si = tlv.SideInfoPoint_str[j];
		std::cout << "[DEBUG] SideInfo #" << j
				<< " SNR: " << si.snr
				<< " Noise: " << si.noise << "\n";
	}

	for (size_t j = 0; j < tlv.RangeProfilePoint_str.size(); ++j) {
		const RangeProfilePoint& rp = tlv.RangeProfilePoint_str[j];
		std::cout << "[PEAK] Bin #" << rp.bin_u16
				<< " | Range: " << rp.range_f << " m"
				<< " | Power: " << rp.power_u16 << "\n";
	}
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

std::vector<TLVPayloadData> SensorData::getTLVPayloadData() 
{
    return payload_data_vect;
}
