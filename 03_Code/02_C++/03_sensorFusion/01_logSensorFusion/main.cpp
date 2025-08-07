/* main.cpp
 *
 * MISRA C++:2008 Compliant Example
 * Demonstrates synchronized radar+IMU logging with 2×N IMU samples per N valid radar points.
 * - threadIwr6843Right(): Reads mmWave frames, filters VALID points.
 * - threadMti710(): Reads Xsens IMU samples.
 * - threadWriter(): For each radar frame with N points, collects 2·N IMU samples and logs.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <cmath>
#include <cstdint>
#include <unistd.h>

#include "mmWave-IWR6843/radar_sensor/IWR6843.h"
#include "mmWave-IWR6843/radar_sensor/SensorData.h"
#include "MTi-G-710/xsens_mti710.hpp"

using namespace std;

/*
Macro to enable or disable sensors
 1 - enable only IWR
 2 - enable only MTI
 3 - enable both
*/ 
#define ENABLE_SENSORS 3

const char CSV_TAB = ',';

/* Uncomment to PRINT instead of logging to CSV */
// #define VALIDATE_PRINT

/*=== Shared Data Structures ===*/

/* Holds one VALID radar detection */
struct ValidRadarPoint
{
    uint32_t frameId;
    uint32_t pointId;
    float    x;
    float    y;
    float    z;
    float    doppler;
    uint16_t snr;
    uint16_t noise;
};

/* Global synchronization objects */
static mutex                   radarMutexRight; /* Protects radarQueueRight  */
static mutex                   radarMutexLeft;  /* Protects radarQueueLeft   */
static mutex                   imuMutex;        /* Protects imuQueue    */
static mutex                   writeMutex;      /* Protects writer sync */
static condition_variable      dataCV;          /* Signals data ready   */

/* Queues for inter-thread communication */
static queue<vector<ValidRadarPoint>> radarQueueRight;
static queue<vector<ValidRadarPoint>> radarQueueLeft;
static queue<MTiData>                 imuQueue;

/* Sensor objects */
static IWR6843     _radarSensorRight;
static IWR6843     _radarSensorLeft;
static XsensMti710 imuSensor;

#ifndef VALIDATE_PRINT
static ofstream csvRadarRight("_outFiles/radarRight_hallway_1.csv");
static ofstream csvRadarLeft("_outFiles/radarLeft_hallway_1.csv");
static ofstream csvImu  ("_outFiles/imu_hallway_1.csv");
#endif

const int UPDATE_POWER = 2000U; /* Minimum peak power for VALID radar points */
const float BIN_TOLERANCE = 0.2;

/*=== threadIwr6843Right(): Radar acquisition & filtering ===*/
void threadIwr6843Right(void)
{
    for (;;)
    {
        int32_t cnt = _radarSensorRight.poll();
        if (cnt < 0)
        {
            cerr << "[ERROR] _radarSensorRight.poll() failed\n";
            break;
        }
        if (cnt == 0)
        {
            usleep(1000);
            continue;
        }

        vector<SensorData> frames;
        if (!_radarSensorRight.copyDecodedFramesFromTop(frames, cnt, true, 100))
        {
            cerr << "[ERROR] Timeout copying radar frames\n";
            continue;
        }

        /* For each decoded frame */
        for (const SensorData& frame : frames)
        {
            /* Step 1: extract frame ID */
            const Frame_header hdr = frame.getHeader(); /* getHeader() is const */
            const uint32_t fid = hdr.getFrameNumber();

            /* Step 2: process each TLV payload block */
            for (const TLVPayloadData& pd : frame.getTLVPayloadData())
            {
                if (pd.SideInfoPoint_str.size() != pd.DetectedPoints_str.size())
                {
                    cerr << "[ERROR] Mismatch: Detected="
                         << pd.DetectedPoints_str.size()
                         << " vs SideInfo="
                         << pd.SideInfoPoint_str.size()
                         << "\n";
                    continue;
                }

                /* Step 3: collect VALID points */
                vector<ValidRadarPoint> validPoints;
                for (size_t i = 0UL; i < pd.DetectedPoints_str.size(); ++i)
                {
                    const DetectedPoints& dp = pd.DetectedPoints_str[i];
                    const float pt_range = sqrtf(dp.x_f * dp.x_f +
                                                 dp.y_f * dp.y_f +
                                                 dp.z_f * dp.z_f);

                    /* find closest peak */
                    float closest_range = -1.0F;
                    uint16_t closest_power = 0U;
                    float    min_diff = numeric_limits<float>::max();
                    for (const RangeProfilePoint& rp : pd.RangeProfilePoint_str)
                    {
                        const float diff = fabsf(pt_range - rp.range_f);
                        if (diff < min_diff)
                        {
                            min_diff       = diff;
                            closest_range  = rp.range_f;
                            closest_power  = rp.power_u16;
                        }
                    }

                    /* validity test */
                    // After finding closest_peak_range, min_diff and closest_peak_power:
                    const bool is_valid =
                        ((closest_range >= 0.0F)                   // Ensure we actually found a peak
                        &&                                          
                        (min_diff      < BIN_TOLERANCE)            // Range‐bin tolerance:  
                                                                   //    • 0.047 m/bin ⇒ ±2 bins ≈0.1 m  
                                                                   //    • bump to 0.2 m for ±4 bins if targets wander  
                                                                   //    • tighten (e.g. 0.1 m) if you know objects are point-like
                        &&                                          
                        (closest_power > UPDATE_POWER));           // Minimum peak power:  
                                                                   //    • this is raw magnitude squared  
                                                                   //    • must exceed CFAR threshold (mean_noise + K)  
                                                                   //    • lower toward 2000–2800U if you’re losing weak targets  
                                                                   //    • raise if too many false detections in clutter

                    if (is_valid && (i < pd.SideInfoPoint_str.size()))
                    {
                        const SideInfoPoint& si = pd.SideInfoPoint_str[i];
                        validPoints.push_back(ValidRadarPoint
                        {
                            fid,
                            static_cast<uint32_t>(i + 1U),
                            dp.x_f,
                            dp.y_f,
                            dp.z_f,
                            dp.doppler_f,
                            si.snr,
                            si.noise
                        });
                    }
                }

                /* Step 4: enqueue if non-empty */
                if (!validPoints.empty())
                {
                    {
                        lock_guard<mutex> lock(radarMutexRight);
                        radarQueueRight.push(move(validPoints));
                    }
                    dataCV.notify_one();
                }
            }
        }
    }
}

/*=== threadIwr6843Left(): Radar acquisition & filtering ===*/
void threadIwr6843Left(void)
{
    for (;;)
    {
        int32_t cnt = _radarSensorLeft.poll();
        if (cnt < 0)
        {
            cerr << "[ERROR] _radarSensorLeft.poll() failed\n";
            break;
        }
        if (cnt == 0)
        {
            usleep(1000);
            continue;
        }

        vector<SensorData> frames;
        if (!_radarSensorLeft.copyDecodedFramesFromTop(frames, cnt, true, 100))
        {
            cerr << "[ERROR] Timeout copying radar frames\n";
            continue;
        }

        /* For each decoded frame */
        for (const SensorData& frame : frames)
        {
            /* Step 1: extract frame ID */
            const Frame_header hdr = frame.getHeader(); /* getHeader() is const */
            const uint32_t fid = hdr.getFrameNumber();

            /* Step 2: process each TLV payload block */
            for (const TLVPayloadData& pd : frame.getTLVPayloadData())
            {
                if (pd.SideInfoPoint_str.size() != pd.DetectedPoints_str.size())
                {
                    cerr << "[ERROR] Mismatch: Detected="
                         << pd.DetectedPoints_str.size()
                         << " vs SideInfo="
                         << pd.SideInfoPoint_str.size()
                         << "\n";
                    continue;
                }

                /* Step 3: collect VALID points */
                vector<ValidRadarPoint> validPoints;
                for (size_t i = 0UL; i < pd.DetectedPoints_str.size(); ++i)
                {
                    const DetectedPoints& dp = pd.DetectedPoints_str[i];
                    const float pt_range = sqrtf(dp.x_f * dp.x_f +
                                                 dp.y_f * dp.y_f +
                                                 dp.z_f * dp.z_f);

                    /* find closest peak */
                    float closest_range = -1.0F;
                    uint16_t closest_power = 0U;
                    float    min_diff = numeric_limits<float>::max();
                    for (const RangeProfilePoint& rp : pd.RangeProfilePoint_str)
                    {
                        const float diff = fabsf(pt_range - rp.range_f);
                        if (diff < min_diff)
                        {
                            min_diff       = diff;
                            closest_range  = rp.range_f;
                            closest_power  = rp.power_u16;
                        }
                    }

                    /* validity test */
                    // After finding closest_peak_range, min_diff and closest_peak_power:
                    const bool is_valid =
                        ((closest_range >= 0.0F)                   // Ensure we actually found a peak
                        &&                                          
                        (min_diff      < BIN_TOLERANCE)            // Range‐bin tolerance:  
                                                                   //    • 0.047 m/bin ⇒ ±2 bins ≈0.1 m  
                                                                   //    • bump to 0.2 m for ±4 bins if targets wander  
                                                                   //    • tighten (e.g. 0.1 m) if you know objects are point-like
                        &&                                          
                        (closest_power > UPDATE_POWER));           // Minimum peak power:  
                                                                   //    • this is raw magnitude squared  
                                                                   //    • must exceed CFAR threshold (mean_noise + K)  
                                                                   //    • lower toward 2000–2800U if you’re losing weak targets  
                                                                   //    • raise if too many false detections in clutter

                    if (is_valid && (i < pd.SideInfoPoint_str.size()))
                    {
                        const SideInfoPoint& si = pd.SideInfoPoint_str[i];
                        validPoints.push_back(ValidRadarPoint
                        {
                            fid,
                            static_cast<uint32_t>(i + 1U),
                            dp.x_f,
                            dp.y_f,
                            dp.z_f,
                            dp.doppler_f,
                            si.snr,
                            si.noise
                        });
                    }
                }

                /* Step 4: enqueue if non-empty */
                if (!validPoints.empty())
                {
                    {
                        lock_guard<mutex> lock(radarMutexLeft);
                        radarQueueLeft.push(move(validPoints));
                    }
                    dataCV.notify_one();
                }
            }
        }
    }
}

/*=== threadMti710(): IMU acquisition ===*/
void threadMti710(void)
{
    /* Step 2: continuous read/parse */
    xsens_interface_t iface = XSENS_INTERFACE_RX(&XsensMti710::xsens_event_handler);
    uint8_t buf[256];
    for (;;)
    {
        const ssize_t n = read(imuSensor.getFd(), buf, sizeof(buf));
        if (n <= 0)
        {
            cerr << "[ERROR] IMU read error\n";
            break;
        }
        xsens_mti_parse_buffer(&iface, buf, static_cast<size_t>(n));
        const MTiData data = imuSensor.getXsensData();

        /* Step 3: enqueue sample */
        {
            lock_guard<mutex> lock(imuMutex);
            imuQueue.push(data);
        }
        dataCV.notify_one();
    }
}

/*=== threadWriter(): synchronizes and logs data ===*/
void threadWriter(bool enableRadar, bool enableImu)
{
#ifndef VALIDATE_PRINT
    if ((enableRadar && !csvRadarRight.is_open()) || 
        (enableRadar && !csvRadarLeft.is_open()) || 
        (enableImu && !csvImu.is_open()))
    {
        cerr << "[ERROR] Output files not open!\n";
        return;
    }

    if (enableRadar)
    {
        csvRadarRight << "frame_id,point_id,x,y,z,doppler,snr,noise\n";
        csvRadarLeft << "frame_id,point_id,x,y,z,doppler,snr,noise\n";
    }
    if (enableImu)
    {
        csvImu   << "frame_id,imu_idx,"
                 << "quat_w,quat_x,quat_y,quat_z,"
                 << "accel_x,accel_y,accel_z,"
                 << "free_accel_x,free_accel_y,free_accel_z,"
                 << "delta_v_x,delta_v_y,delta_v_z,"
                 << "delta_q_w,delta_q_x,delta_q_y,delta_q_z,"
                 << "rate_x,rate_y,rate_z,"
                 << "quat_w,quat_x,quat_y,quat_z,"
                 << "mag_x,mag_y,mag_z,"
                 << "temperature,status_byte,packet_counter,time_fine\n";
    }
#else
    cout << "[VALIDATE_PRINT] Writer in PRINT mode\n";
#endif

    for (;;)
    {
        /* === Wait for radar data === */
        unique_lock<mutex> radarLock(radarMutexRight);
        dataCV.wait(radarLock, [&]{
            return !radarQueueRight.empty();
        });

        /* === Try to acquire radar data from RIGHT sensor === */
        vector<ValidRadarPoint> radarPtsRight;
        {
            unique_lock<mutex> radarLock(radarMutexRight);
            if (!radarQueueRight.empty())
            {
                radarPtsRight = move(radarQueueRight.front());
                radarQueueRight.pop();
            }
        }

        /* === Try to acquire radar data from LEFT sensor === */
        vector<ValidRadarPoint> radarPtsLeft;
        {
            unique_lock<mutex> radarLock(radarMutexLeft);
            if (!radarQueueLeft.empty())
            {
                radarPtsLeft = move(radarQueueLeft.front());
                radarQueueLeft.pop();
            }
        }

        /* === If no radar data available, wait briefly === */
        if (radarPtsRight.empty() && radarPtsLeft.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        const size_t N = radarPtsRight.size() + radarPtsLeft.size();
        uint32_t fid = 0U;
        if (!radarPtsRight.empty())
        {
            fid = radarPtsRight.front().frameId;
        }
        else if (!radarPtsLeft.empty())
        {
            fid = radarPtsLeft.front().frameId;
        }

        /* === Collect IMU samples only if enabled === */
        vector<MTiData> imuSamples;
        if (enableImu)
        {
            const size_t imuNeeded = N * 2UL;
            unique_lock<mutex> imuLock(imuMutex);
            dataCV.wait(imuLock, [&]{
                return imuQueue.size() >= imuNeeded;
            });

            imuSamples.reserve(imuNeeded);
            for (size_t i = 0UL; i < imuNeeded; ++i)
            {
                imuSamples.push_back(imuQueue.front());
                imuQueue.pop();
            }
            imuLock.unlock();
        }

        /* === Write radar data === */
        if (enableRadar)
        {
#ifdef VALIDATE_PRINT
            for (const auto& pt : radarPtsRight)
            {
                cout << "[RADAR] frame=" << pt.frameId
                     << " pt="   << pt.pointId
                     << " x="    << pt.x
                     << " y="    << pt.y
                     << " z="    << pt.z
                     << " dop="  << pt.doppler
                     << " snr="  << pt.snr
                     << " noise="<< pt.noise
                     << "\n";
            }
            for (const auto& pt : radarPtsLeft)
            {
                cout << "[RADAR] frame=" << pt.frameId
                     << " pt="   << pt.pointId
                     << " x="    << pt.x
                     << " y="    << pt.y
                     << " z="    << pt.z
                     << " dop="  << pt.doppler
                     << " snr="  << pt.snr
                     << " noise="<< pt.noise
                     << "\n";
            }
#else
            for (const auto& pt : radarPtsLeft)
            {
                csvRadarLeft << pt.frameId  << CSV_TAB
                         << pt.pointId  << CSV_TAB
                         << pt.x        << CSV_TAB
                         << pt.y        << CSV_TAB
                         << pt.z        << CSV_TAB
                         << pt.doppler  << CSV_TAB
                         << pt.snr      << CSV_TAB
                         << pt.noise    << "\n";
            }
            csvRadarLeft.flush();
#endif
        }

        /* === Write IMU data if enabled === */
        if (enableImu)
        {
            for (size_t i = 0UL; i < imuSamples.size(); ++i)
            {
                const MTiData& imu = imuSamples[i];
#ifdef VALIDATE_PRINT
                cout << "[IMU] frame=" << fid
                     << " idx="  << (i + 1UL)
                     << " accel=("
                     << imu.acceleration[0] << ","
                     << imu.acceleration[1] << ","
                     << imu.acceleration[2] << ")  pkt="
                     << imu.packet_counter
                     << "\n";
#else
                csvImu << fid               << CSV_TAB
                       << (i + 1UL)         << CSV_TAB
                       << imu.quaternion[0] << CSV_TAB
                       << imu.quaternion[1] << CSV_TAB
                       << imu.quaternion[2] << CSV_TAB
                       << imu.quaternion[3] << CSV_TAB
                       << imu.acceleration[0]      << CSV_TAB
                       << imu.acceleration[1]      << CSV_TAB
                       << imu.acceleration[2]      << CSV_TAB
                       << imu.free_acceleration[0] << CSV_TAB
                       << imu.free_acceleration[1] << CSV_TAB
                       << imu.free_acceleration[2] << CSV_TAB
                       << imu.delta_v[0]           << CSV_TAB
                       << imu.delta_v[1]           << CSV_TAB
                       << imu.delta_v[2]           << CSV_TAB
                       << imu.delta_q[0]           << CSV_TAB
                       << imu.delta_q[1]           << CSV_TAB
                       << imu.delta_q[2]           << CSV_TAB
                       << imu.delta_q[3]           << CSV_TAB
                       << imu.rate_of_turn[0]      << CSV_TAB
                       << imu.rate_of_turn[1]      << CSV_TAB
                       << imu.rate_of_turn[2]      << CSV_TAB
                       << imu.quaternion[0]        << CSV_TAB
                       << imu.quaternion[1]        << CSV_TAB
                       << imu.quaternion[2]        << CSV_TAB
                       << imu.quaternion[3]        << CSV_TAB
                       << imu.magnetic[0]          << CSV_TAB
                       << imu.magnetic[1]          << CSV_TAB
                       << imu.magnetic[2]          << CSV_TAB
                       << imu.temperature          << CSV_TAB
                       << int(imu.status_byte)     << CSV_TAB
                       << imu.packet_counter       << CSV_TAB
                       << imu.time_fine            << "\n";
            }
            csvImu.flush();
#endif
        }
    }
}

/*=== MAIN ===*/
int main(void)
{
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    cout << "[INFO] Initializing radar...\n";
    if (_radarSensorRight.init("/dev/ttyUSB0",
                         "/dev/ttyUSB1",
                         "../01_logSensorFusion/mmWave-IWR6843/configs/"
                         "profile_azim60_elev30_calibrator.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] _radarSensorRight.init() failed\n";
        return 1;
    }
    if (_radarSensorLeft.init("/dev/ttyUSB2",
                         "/dev/ttyUSB3",
                         "../01_logSensorFusion/mmWave-IWR6843/configs/"
                         "profile_azim60_elev30_calibrator.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] _radarSensorLeft.init() failed\n";
        return 1;
    }
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    if ((imuSensor.findXsensDevice() != DEVICE_FOUND_SUCCESS) ||
        (imuSensor.openXsensPort()   != OPEN_PORT_SUCCESS))
    {
        cerr << "[ERROR] Unable to initialize IMU\n";
        return 1;
    }
    imuSensor.configure();
    #endif

#ifndef VALIDATE_PRINT
    cout << "[INFO] Opening output files...\n";
    if ((!csvRadarRight.is_open()) || (!csvRadarLeft.is_open()) || (!csvImu.is_open()))
    {
        cerr << "[ERROR] Failed to open CSVs\n";
        return 1;
    }
#endif

    cout << "[INFO] Spawning threads...\n";
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread thread_iwr6843_right(threadIwr6843Right);
    thread thread_iwr6843_left(threadIwr6843Left);
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread thread_mti710(threadMti710);
    #endif
    thread thread_logger(threadWriter, 
                        (ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3),
                        (ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3));

    /* Join threads (program runs until killed) */
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread_iwr6843_right.join();
    thread_iwr6843_left.join();
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread_mti710.join();
    #endif
    thread_logger.join();

#ifndef VALIDATE_PRINT
    csvRadarRight.close();
    csvRadarLeft.close();
    csvImu.close();
#endif

    return 0;
}
