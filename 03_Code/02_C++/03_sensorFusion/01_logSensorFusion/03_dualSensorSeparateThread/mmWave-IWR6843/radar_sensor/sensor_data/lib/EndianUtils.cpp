// EndianUtils.cpp
#include "EndianUtils.h"
#include <stdexcept>
#include <cmath>

uint16_t EndianUtils::toLittleEndian16(std::vector<uint8_t>& data, uint8_t bytesToCheck)
{
    if (data.size() < bytesToCheck || bytesToCheck > 2) {
        throw std::invalid_argument("16 Invalid number of bytes to process");
    }

    uint16_t result = 0;
    for (uint8_t i = 0; i < bytesToCheck; ++i) {
        result |= static_cast<uint16_t>(data[i]) << (8 * i);
    }

    data.erase(data.begin(), data.begin() + bytesToCheck);
    return result;
}


uint32_t EndianUtils::toLittleEndian32(std::vector<uint8_t>& data, uint8_t bytesToCheck)
{
    // Ensure the vector has at least the required number of bytes
    if (data.size() < bytesToCheck || bytesToCheck > 4) {
        throw std::invalid_argument("32 Invalid number of bytes to process");
    }

    uint32_t result = 0;

    // Extract little-endian 32-bit value from the vector
    for (uint8_t i = 0; i < bytesToCheck; ++i) {
        result |= static_cast<uint32_t>(data[i]) << (8 * i);
    }

    // Remove the used bytes from the beginning of the vector
    data.erase(data.begin(), data.begin() + bytesToCheck);

    return result;
}

float EndianUtils::toFloat32(uint32_t var32)
{
    // Reinterpret the bits of the 32-bit integer as a float
    float floatValue;
    std::memcpy(&floatValue, &var32, sizeof(float));

    return floatValue;
}

uint64_t EndianUtils::toLittleEndian64(std::vector<uint8_t>& data, uint8_t bytesToCheck)
{
    // Ensure the vector has at least the required number of bytes
    if (data.size() < bytesToCheck || bytesToCheck > 8) {
        throw std::invalid_argument("64 Invalid number of bytes to process");
    }

    uint64_t result = 0;

    // Extract little-endian 64-bit value from the vector
    for (uint8_t i = 0; i < bytesToCheck; ++i) {
        result |= static_cast<uint64_t>(data[i]) << (8 * i);
    }

    // Remove the used bytes from the beginning of the vector
    data.erase(data.begin(), data.begin() + bytesToCheck);

    return result;
}

// Function to round a float to a specified number of decimal places
float EndianUtils::roundToPrecision(float value, int precision) {
    float scale = std::pow(10.0f, precision);
    return std::round(value * scale) / scale;
}