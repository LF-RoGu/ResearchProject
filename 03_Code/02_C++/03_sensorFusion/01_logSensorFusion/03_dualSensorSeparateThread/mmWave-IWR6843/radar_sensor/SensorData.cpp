#include "SensorData.h"
#include <iomanip>
//#define DEBUG_FRAME_TLV

SensorData::SensorData()
{
}

SensorData::SensorData(std::vector<uint8_t> rawData)
{	
    // Define the expected 8-byte magic word pattern
    static const std::vector<uint8_t> magicBytes = {
        0x02, 0x01, 0x04, 0x03,
        0x06, 0x05, 0x08, 0x07
    };

    // Attempt to find the start of a valid frame by locating the magic word
    auto it = std::search(rawData.begin(), rawData.end(), magicBytes.begin(), magicBytes.end());

    if (it == rawData.end()) {
        std::cerr << "[ERROR] Magic word not found in raw data. Skipping.\n";
        return;
    }

    // Extract aligned data starting from the magic word
    std::vector<uint8_t> alignedData(it, rawData.end());

    // Store raw data for possible debugging/logging
    storedRawData.reserve(alignedData.size());
    std::copy(alignedData.begin(), alignedData.end(), std::back_inserter(storedRawData));

    // Basic safety check: ensure at least 44 bytes for header and magic word
    if (alignedData.size() < 44) {
        std::cerr << "[WARNING] Frame too short after alignment. Skipping.\n";
        return;
    }

    // Parse the frame header
    header = Frame_header(alignedData);

    if (!header.isValid()) {
        std::cerr << "[WARNING] Invalid frame header. Skipping.\n";
        return;
    } else {
        //std::cout << "[INFO] Frame parsed successfully.\n";
    }

    // Parse TLV payload
    payload = TLV_payload(alignedData, header.getNumObjDetecter(), header.getNumTLV());
    payload_data_vect = payload.getTLVPayloadData();
}

Frame_header SensorData::getHeader() const
{
	return header;
}

TLV_payload SensorData::getTLVPayload() const
{
	return payload;
}

std::vector<TLVPayloadData> SensorData::getTLVPayloadData() const
{
    return payload_data_vect;
}
