#include "UARTFrame.h"
#include <iomanip>  // Needed for std::setprecision

//#define DEBUG_UART_FRAME
//#define DEBUG_UART_FRAME_TLV_DATA


constexpr uint64_t MAGIC_WORD = 0x0708050603040102;

UART_frame::UART_frame() {}


Frame_header::Frame_header()
{
}

Frame_header::Frame_header(std::vector<uint8_t>& data)
{
    parseFrameHeader(data);
}

void Frame_header::parseFrameHeader(std::vector<uint8_t>& data)
{
    EndianUtils EndianUtils_c;

    // Extract magic word (64-bit) from the vector
    uint64_t magicWord = EndianUtils_c.toLittleEndian64(data, 8);

    // Check if the magic word matches the expected value
    if (magicWord != MAGIC_WORD) {
        std::cerr << "Error: Invalid magic word detected! Aborting frame parsing.\n";
        frameValid = false;
        return; // Early exit if the magic word is invalid
    }
    frameValid = true;

    // Extract MagicWord (64-bit) from the vector
    setMagicWord(magicWord);

    // Extract version (32-bit) from the vector
    setVersion(EndianUtils_c.toLittleEndian32(data, 4));

    // Extract packet length (32-bit) from the vector
    setPacketLength(EndianUtils_c.toLittleEndian32(data, 4));

    // Extract platform (32-bit) from the vector
    setPlatform(EndianUtils_c.toLittleEndian32(data, 4));

    // Extract frame number (32-bit) from the vector
    setFrameNumber(EndianUtils_c.toLittleEndian32(data, 4));

    // Extract time (32-bit) from the vector
    setTime(EndianUtils_c.toLittleEndian32(data, 4));

    // Extract number of detected objects (32-bit) from the vector
    setNumObjDetecter(EndianUtils_c.toLittleEndian32(data, 4));

    // Extract number of TLVs (32-bit) from the vector
    setNumTLV(EndianUtils_c.toLittleEndian32(data, 4));

    // Extract subframe number (32-bit) from the vector
    setSubframeNum(EndianUtils_c.toLittleEndian32(data, 4));
}

void Frame_header::setMagicWord(uint64_t var)
{
    FrameHeader_str.magicWord_u64 = var;
}

void Frame_header::setVersion(uint32_t var)
{
    FrameHeader_str.version_u32 = var;
}

void Frame_header::setPacketLength(uint32_t var)
{
    FrameHeader_str.totalPacketLength_u32 = var;
}

void Frame_header::setPlatform(uint32_t var)
{
    FrameHeader_str.platform_u32 = var;
}

void Frame_header::setFrameNumber(uint32_t var)
{
    FrameHeader_str.frameNumber_u32 = var;
}

void Frame_header::setTime(uint32_t var)
{
    FrameHeader_str.timeCpuCycles_u32 = var;
}

void Frame_header::setNumObjDetecter(uint32_t var)
{
    FrameHeader_str.numDetectedObj_u32 = var;
}

void Frame_header::setNumTLV(uint32_t var)
{
    FrameHeader_str.numTLVs_u32 = var;
}


void Frame_header::setSubframeNum(uint32_t var)
{
    FrameHeader_str.subFrameNumber_u32 = var;
}

uint64_t Frame_header::getMagicWord() const
{
    return FrameHeader_str.magicWord_u64;
}

uint32_t Frame_header::getVersion() const
{
    return FrameHeader_str.version_u32;
}

uint32_t Frame_header::getPacketLength() const
{
    return FrameHeader_str.totalPacketLength_u32;
}

uint32_t Frame_header::getPlatform() const
{
    return FrameHeader_str.platform_u32;
}

uint32_t Frame_header::getFrameNumber() const
{
    return FrameHeader_str.frameNumber_u32;
}

uint32_t Frame_header::getTime() const
{
    return FrameHeader_str.timeCpuCycles_u32;
}

uint32_t Frame_header::getNumObjDetecter() const
{
    return FrameHeader_str.numDetectedObj_u32;
}

uint32_t Frame_header::getNumTLV() const
{
    return FrameHeader_str.numTLVs_u32;
}

uint32_t Frame_header::getSubframeNum() const
{
    return FrameHeader_str.subFrameNumber_u32;
}


TLV_frame::TLV_frame() 
{
    
}

TLV_frame::TLV_frame(std::vector<uint8_t>& data, uint32_t numDetectedObj_var)
{
    TLVHeaderData_str = parseTLVHeader(data);
    TLVPayloadData_str = parseTLVPayload(data, TLVHeaderData_str, numDetectedObj_var);
}

TLVHeaderData TLV_frame::parseTLVHeader(std::vector<uint8_t>& data)
{
    EndianUtils EndianUtils_c;
    TLV_header TLV_header_c;
    TLVHeaderData TLVHeaderData_str;

    TLV_header_c.setType(EndianUtils_c.toLittleEndian32(data,4));
    TLV_header_c.setLength(EndianUtils_c.toLittleEndian32(data, 4));

    TLVHeaderData_str.type_u32 = TLV_header_c.getType();
    TLVHeaderData_str.length_u32 = TLV_header_c.getLength();
    return TLVHeaderData_str;
}

TLVPayloadData TLV_frame::parseTLVPayload(std::vector<uint8_t>& data, TLVHeaderData TLVHeaderData_var, uint32_t numDetectedObj_var)
{
    EndianUtils EndianUtils_c;
    TLVPayloadData TLVPayloadData_str;
    // Implement parsing logic based on the type and length from the header
    TLVHeaderData_var.type_u32;
    TLVHeaderData_var.length_u32;

    switch (TLVHeaderData_var.type_u32) 
    {
    case 1: // Detected points
    {
        DetectedPoints DetectedPoints_temp;
        for (uint32_t i = 0; i < numDetectedObj_var; i++)
        {
            // Parse x, y, z, and doppler values (4 bytes each)
            uint32_t x_int = EndianUtils::toLittleEndian32(data, 4);
            uint32_t y_int = EndianUtils::toLittleEndian32(data, 4);
            uint32_t z_int = EndianUtils::toLittleEndian32(data, 4);
            uint32_t doppler_int = EndianUtils::toLittleEndian32(data, 4);

            // Convert to float
            float x_f = EndianUtils::toFloat32(x_int);
            float y_f = EndianUtils::toFloat32(y_int);
            float z_f = EndianUtils::toFloat32(z_int);
            float doppler_f = EndianUtils::toFloat32(doppler_int);
#ifdef DEBUG_UART_FRAME
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "Converted Floats:" << std::endl;
            std::cout << "  x: " << x_f << " meters" << std::endl;
            std::cout << "  y: " << y_f << " meters" << std::endl;
            std::cout << "  z: " << z_f << " meters" << std::endl;
            std::cout << "  doppler: " << doppler_f << " m/s" << std::endl;
#endif

            DetectedPoints_temp.x_f = EndianUtils::roundToPrecision(x_f, 4);
            DetectedPoints_temp.y_f = EndianUtils::roundToPrecision(y_f, 4);
            DetectedPoints_temp.z_f = EndianUtils::roundToPrecision(z_f, 4);
            DetectedPoints_temp.doppler_f = EndianUtils::roundToPrecision(doppler_f, 4);

            TLVPayloadData_str.DetectedPoints_str.push_back(DetectedPoints_temp);
        }
    }
    break;
    case 2: // Range Profile with rolling noise floor
    {
        uint32_t numBins = (TLVHeaderData_var.length_u32 - 8) / 2;
        #ifdef DEBUG_UART_FRAME
        std::cout << "[DEBUG] Range Profile: " << numBins << " bins" << std::endl;
        #endif

        float range_resolution_m = 0.047f; // from your config

        // 1) Read all bins into a temp vector
        std::vector<uint16_t> allBins;
        for (uint32_t i = 0; i < numBins; ++i)
        {
            if (data.size() < 2) {
                std::cerr << "[ERROR] Not enough bytes for Range Profile bin!" << std::endl;
                break;
            }
            uint16_t binValue = EndianUtils_c.toLittleEndian16(data, 2);
            allBins.push_back(binValue);
        }

        // 2) Compute mean noise floor
        uint64_t sum = 0;
        for (auto val : allBins) {
            sum += val;
        }
        float mean_floor = static_cast<float>(sum) / allBins.size();
        /*
        TODO: Check K for peaks
        */
        float K = 500.0f;  // adjust based on your testing
        float adaptive_threshold = mean_floor + K;
        #ifdef DEBUG_UART_FRAME
        std::cout << "[DEBUG] Mean noise floor: " << mean_floor 
                << " | Adaptive threshold: " << adaptive_threshold << std::endl;
        #endif

        // 3) Store only bins above threshold
        for (uint32_t i = 0; i < allBins.size(); ++i)
        {
            uint16_t power = allBins[i];
            if (power >= adaptive_threshold)
            {
                RangeProfilePoint rp;
                rp.bin_u16 = i;
                rp.range_f = i * range_resolution_m;
                rp.power_u16 = power;

                TLVPayloadData_str.RangeProfilePoint_str.push_back(rp);
    #ifdef DEBUG_UART_FRAME
                std::cout << "[PEAK] Bin #" << rp.bin_u16 
                        << " | Range: " << rp.range_f 
                        << " m | Power: " << rp.power_u16 << std::endl;
    #endif
            }
        }
    }
    break;

    case 3: // Noise Profile
    {

    }
    break;
    case 4: // Azimuth Static Heatmap
    {

    }
    break;
    case 7: // Side Info for Detected Points
    {
        for (uint32_t i = 0; i < numDetectedObj_var; i++) {
            SideInfoPoint sideInfo;
            sideInfo.snr   = EndianUtils_c.toLittleEndian16(data, 2);
            sideInfo.noise = EndianUtils_c.toLittleEndian16(data, 2);
            TLVPayloadData_str.SideInfoPoint_str.push_back(sideInfo);
        #ifdef DEBUG_UART_FRAME
            std::cout << "[DEBUG] SideInfo #" << i 
                    << " SNR: " << sideInfo.snr
                    << " Noise: " << sideInfo.noise << std::endl;
        #endif
        }
    }
    break;
    default:
    {
        // ** NEW: skip this entire TLV payload **
        // length includes header (8 bytes) + payload bytes
        uint32_t payloadBytes = TLVHeaderData_var.length_u32 - 8;
        if (data.size() < payloadBytes) {
            std::cerr << "[ERROR] Not enough bytes to skip unknown TLV (type="
                        << TLVHeaderData_var.type_u32 << ")\n";
            break;
        }
        // erase the payload bytes so the next TLVHeader starts in sync
        data.erase(data.begin(),
                    data.begin() + payloadBytes);
        std::cout << "[INFO] Skipped TLV type="
                    << TLVHeaderData_var.type_u32
                    << " (" << payloadBytes << " bytes)\n";
    }
    break;
    }
    return TLVPayloadData_str;
}

TLVHeaderData TLV_frame::getTLVFrameHeaderData()
{
    return TLVHeaderData_str;
}

TLVPayloadData TLV_frame::getTLVFramePayloadData()
{
    return TLVPayloadData_str;
}

TLV_header::TLV_header()
{
}

void TLV_header::setType(uint32_t var)
{
    TLVHeaderData_str.type_u32 = var;
}

void TLV_header::setLength(uint32_t var)
{
    TLVHeaderData_str.length_u32 = var;
}

uint32_t TLV_header::getType() const
{
    return TLVHeaderData_str.type_u32;
}

uint32_t TLV_header::getLength() const
{
    return TLVHeaderData_str.length_u32;
}

TLV_payload::TLV_payload()
{
}

// Constructor implementation
TLV_payload::TLV_payload(std::vector<uint8_t>& data, uint32_t numDetectedObj_var, uint32_t numTLVs_var)
{
    TLVPayloadData mergedPayloadData;

    #ifdef DEBUG_UART_FRAME_TLV_DATA
    std::cout << "\n[DEBUG] Entering TLV_payload() loop for TLVs" << std::endl;
    std::cout << "  Num Detected Objects: " << numDetectedObj_var << std::endl;
    std::cout << "  Num TLVs: " << numTLVs_var << std::endl;
    std::cout << "  Buffer size at start: " << data.size() << std::endl;
    #endif
    for (uint32_t i = 0; i < numTLVs_var; ++i)
    {
        #ifdef DEBUG_UART_FRAME_TLV_DATA
        std::cout << "\n[DEBUG] === TLV #" << i << " ===" << std::endl;
        #endif

        TLV_frame frame(data, numDetectedObj_var);
        TLVHeaderData tlvHeader = frame.getTLVFrameHeaderData();

        #ifdef DEBUG_UART_FRAME_TLV_DATA
        std::cout << "[DEBUG] Parsed TLV Header → Type: " << tlvHeader.type_u32
                << " | Length: " << tlvHeader.length_u32
                << " | Buffer left: " << data.size() << std::endl;
        #endif

        TLVPayloadData tlvPayloadData = frame.getTLVFramePayloadData();

        #ifdef DEBUG_UART_FRAME_TLV_DATA
        std::cout << "[DEBUG] Points: " << tlvPayloadData.DetectedPoints_str.size()
                << " | SideInfo: " << tlvPayloadData.SideInfoPoint_str.size() << std::endl;
        #endif

        mergedPayloadData.DetectedPoints_str.insert(
            mergedPayloadData.DetectedPoints_str.end(),
            tlvPayloadData.DetectedPoints_str.begin(),
            tlvPayloadData.DetectedPoints_str.end()
        );
        mergedPayloadData.SideInfoPoint_str.insert(
            mergedPayloadData.SideInfoPoint_str.end(),
            tlvPayloadData.SideInfoPoint_str.begin(),
            tlvPayloadData.SideInfoPoint_str.end()
        );
        mergedPayloadData.RangeProfilePoint_str.insert(
            mergedPayloadData.RangeProfilePoint_str.end(),
            tlvPayloadData.RangeProfilePoint_str.begin(),
            tlvPayloadData.RangeProfilePoint_str.end()
        );   
    }
    setTLVPayloadData(mergedPayloadData);
}


void TLV_payload::setTLVPayloadData(TLVPayloadData TLVPayloadData_var)
{
    TLVPayloadData_vect.push_back(TLVPayloadData_var);
}

std::vector<TLVPayloadData> TLV_payload::getTLVPayloadData()
{
    return TLVPayloadData_vect;
}
