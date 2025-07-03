#include "SensorData.h"

//#define DEBUG_FRAME_TLV

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

	payload = TLV_payload(rawData, header.getNumObjDetecter(), header.getNumTLV());
	payload_data_vect = payload.getTLVPayloadData();

	#ifdef DEBUG_FRAME_TLV
	std::cout << "\n================== SENSOR DATA DEBUG ==================\n";
	for (size_t i = 0; i < payload_data_vect.size(); ++i) {
	const TLVPayloadData& tlv = payload_data_vect[i];

	for (const TLVPayloadData& pd : payload_data_vect)
	{
		// Detected Points
		for (const DetectedPoints& dp : pd.DetectedPoints_str) {
			std::cout << "Converted Floats:\n";
			std::cout << "  x: " << dp.x_f << " meters\n";
			std::cout << "  y: " << dp.y_f << " meters\n";
			std::cout << "  z: " << dp.z_f << " meters\n";
			std::cout << "  doppler: " << dp.doppler_f << " m/s\n";
		}

		// Side Info Points
		for (size_t j = 0; j < pd.SideInfoPoint_str.size(); ++j) {
			const SideInfoPoint& si = pd.SideInfoPoint_str[j];
			std::cout << "[DEBUG] SideInfo #" << j
					<< " SNR: " << si.snr
					<< " Noise: " << si.noise << "\n";
		}

		// Range Profile Points
		for (const RangeProfilePoint& rp : pd.RangeProfilePoint_str) {
			std::cout << "[PEAK] Bin #" << rp.bin_u16
					<< " | Range: " << rp.range_f << " m"
					<< " | Power: " << rp.power_u16 << "\n";
		}
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

std::vector<TLVPayloadData> SensorData::getTLVPayloadData() const
{
    return payload_data_vect;
}
