#ifndef SENSORDATASTR_H
#define SENSORDATASTR_H

#include <cstdint>
#include <vector>
#include <iostream>

#define CONTINUOUS_READING 1       // 1 for continuous, 0 for limited frames
#define FIXED_FRAME_COUNT 10       // Number of frames to read if continuous is disabled

struct FrameHeaderData {
    uint64_t magicWord_u64;
    uint32_t version_u32;
    uint32_t totalPacketLength_u32;
    uint32_t platform_u32;
    uint32_t frameNumber_u32;
    uint32_t timeCpuCycles_u32;
    uint32_t numDetectedObj_u32;
    uint32_t numTLVs_u32;
    uint32_t subFrameNumber_u32;
};

struct TLVHeaderData {
    uint32_t type_u32;
    uint32_t length_u32;
};

// Additional structs for various types of detected data
struct DetectedPoints {
    float x_f;
    float y_f;
    float z_f;
    float doppler_f;
};

struct RangeProfilePoint {
    uint16_t rangePoint;
};

struct NoiseProfilePoint {
    uint16_t noisePoint;
};

struct AzimuthHeatmapPoint {
    int16_t imag;
    int16_t real;
};

struct SideInfoPoint {
    uint16_t snr;
    uint16_t noise;
};

struct AzimuthElevationHeatmapPoint {
    int16_t imag;
    int16_t real;
};

struct SphericalCoordinate {
    float range;
    float azimuth;
    float elevation;
    float doppler;
};

struct TargetData {
    float tid;
    float posX;
    float posY;
    float posZ;
    float velX;
    float velY;
    float velZ;
    float accX;
    float accY;
    float accZ;
    float ec[9];
    float g;
    float confidenceLevel;
};

struct PointCloudUnit {
    float elevationUnit;
    float azimuthUnit;
    float dopplerUnit;
    float rangeUnit;
    float snrUnit;
};

struct CompressedPoint {
    int8_t elevation;
    int8_t azimuth;
    int16_t doppler;
    uint16_t range;
    uint16_t snr;
};

struct TLVPayloadData
{
    std::vector<DetectedPoints> DetectedPoints_str;
    RangeProfilePoint RangeProfilePoint_str;
    NoiseProfilePoint NoiseProfilePoint_str;
    AzimuthHeatmapPoint AzimutHeatmapPoint_str;
    std::vector<SideInfoPoint> SideInfoPoint_str;
    AzimuthElevationHeatmapPoint AzimuthElevationHeatmapPoint_str;
    SphericalCoordinate SphericalCoordinate_str;
    TargetData TargetData_str;
    PointCloudUnit PointCloudUnit_str;
    CompressedPoint CompressedPoint_str;
};
#endif // SENSORDATASTR_H
