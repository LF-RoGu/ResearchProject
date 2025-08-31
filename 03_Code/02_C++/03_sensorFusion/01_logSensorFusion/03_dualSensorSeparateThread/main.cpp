/* main.cpp
 *
 * MISRA C++:2008 Compliant Example
 * Demonstrates synchronized radar+IMU logging with 2×N IMU samples per N valid radar points.
 * - threadIwr6843(): Reads mmWave frames, filters VALID points.
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
#define ENABLE_SENSORS 1

const char CSV_TAB = ',';

/* Uncomment to PRINT instead of logging to CSV */
/* #define VALIDATE_PRINT */

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
static mutex                   radarMutexA;  /* Protects radarQueueA  */
static mutex                   radarMutexB;  /* Protects radarQueueB  */
static mutex                   imuMutex;    /* Protects imuQueue    */
static mutex                   writeMutex;  /* Protects writer sync */
static condition_variable      dataCV;      /* Signals data ready   */

/* Queues for inter-thread communication */
static queue<vector<ValidRadarPoint>> radarQueueA;
static queue<vector<ValidRadarPoint>> radarQueueB;
static queue<MTiData>                 imuQueue;

/* Sensor objects */
static IWR6843     radarSensorA;
static IWR6843     radarSensorB;
static XsensMti710 imuSensor;

#ifndef VALIDATE_PRINT
static ofstream csvRadar("_outFiles/radar_2GHzConfig2.csv");
static ofstream csvImu  ("_outFiles/imu_2GHzConfig2.csv");
#endif

const int UPDATE_POWER = 1800U; /* Minimum peak power for VALID radar points */
const float BIN_TOLERANCE = 0.4;

static vector<vector<ValidRadarPoint>> extractValidRadarPoints(const vector<SensorData>& frames)
{
    vector<vector<ValidRadarPoint>> validFrames;

    for (const SensorData& frame : frames)
    {
        const Frame_header hdr = frame.getHeader();
        const uint32_t fid = hdr.getFrameNumber();

        for (const TLVPayloadData& payloadData : frame.getTLVPayloadData())
        {
            if (payloadData.SideInfoPoint_str.size() != payloadData.DetectedPoints_str.size())
            {
                cerr << "[ERROR] Mismatch: Detected="
                     << payloadData.DetectedPoints_str.size()
                     << " vs SideInfo="
                     << payloadData.SideInfoPoint_str.size()
                     << "\n";
                continue;
            }

            vector<ValidRadarPoint> validPoints;
            for (size_t i = 0UL; i < pd.DetectedPoints_str.size(); ++i)
            {
                const DetectedPoints& detectedPoints = payloadData.DetectedPoints_str[i];
                const SideInfoPoint& sideInfo = payloadData.SideInfoPoint_str[i];

                validPoints.push_back(ValidRadarPoint{
                    fid,
                    static_cast<uint32_t>(i + 1U),
                    detectedPoints.x_f,
                    detectedPoints.y_f,
                    detectedPoints.z_f,
                    detectedPoints.doppler_f,
                    sideInfo.snr,
                    sideInfo.noise
                });
            }

            if (!validPoints.empty())
            {
                validFrames.push_back(std::move(validPoints));
            }
        }
    }
    return validFrames;
}

/*=== threadIwr6843(): Radar acquisition & filtering ===*/
void threadIwr6843(void)
{
    int32_t RadarCount = 0;
    for (;;)
    {
        // Poll information from both radars
        RadarCount = radarSensorA.poll();
        if (RadarCount < 0)
        {
            cerr << "[ERROR] Radar failed to poll\n";
            break;
        }
        if (RadarCount == 0)
        {
            // Wait for new data
            usleep(1000);
            continue;
        }

        vector<SensorData> RadarFrames;
        if (!radarSensorA.copyDecodedFramesFromTop(RadarFrames, RadarCount, true, 100))
        {
            cerr << "[ERROR] Timeout copying radar frames\n";
            continue;
        }

        const vector<vector<ValidRadarPoint>> FrameBatches = extractValidRadarPoints(RadarFrames); 

        /* For each decoded frame */
        // If the batch is not empty
        if(!FrameBatches.empty())
        {
            {
                lock_guard<mutex> lock(radarMutexA);
                for (auto&& batch : FrameBatches)
                {
                    std::cout << "[DEBUG] Pushed LEFT batch of size: " << batch.size() << "\n";
                    for (const auto& pt : batch)
                    {
                        std::cout << "    [Queue] Frame=" << pt.frameId
                                << " Idx=" << pt.pointId
                                << " x=" << pt.x
                                << " y=" << pt.y
                                << " z=" << pt.z
                                << " doppler=" << pt.doppler
                                << " snr=" << pt.snr
                                << " noise=" << pt.noise << "\n";
                    }
                    radarQueueA.push(std::move(batch));
                }
            }
            dataCV.notify_one();
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
    if ((enableRadar && !csvRadar.is_open()) || 
        (enableImu && !csvImu.is_open()))
    {
        cerr << "[ERROR] Output files not open!\n";
        return;
    }

    if (enableRadar)
    {
        csvRadar << "frame_id,point_id,x,y,z,doppler,snr,noise\n";
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
        unique_lock<mutex> radarLock(radarMutexA);
        dataCV.wait(radarLock, [&]{
            return !radarQueueA.empty();
        });

        vector<ValidRadarPoint> radarPts = move(radarQueueA.front());
        radarQueueA.pop();
        radarLock.unlock();

        const size_t N = radarPts.size();
        const uint32_t fid = radarPts.front().frameId;

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
            for (const auto& pt : radarPts)
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
            for (const auto& pt : radarPts)
            {
                csvRadar << pt.frameId  << CSV_TAB
                         << pt.pointId  << CSV_TAB
                         << pt.x        << CSV_TAB
                         << pt.y        << CSV_TAB
                         << pt.z        << CSV_TAB
                         << pt.doppler  << CSV_TAB
                         << pt.snr      << CSV_TAB
                         << pt.noise    << "\n";
            }
            csvRadar.flush();
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
    if (radarSensorA.init("/dev/ttyUSB0",
                         "/dev/ttyUSB1",
                         "../01_logSensorFusion/mmWave-IWR6843/configs/"
                         "profile_azim60_elev30_optimized.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] radarSensorA.init() failed\n";
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
    if ((!csvRadar.is_open()) || (!csvImu.is_open()))
    {
        cerr << "[ERROR] Failed to open CSVs\n";
        return 1;
    }
#endif

    cout << "[INFO] Spawning threads...\n";
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread thread_iwr6843(threadIwr6843);
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread thread_mti710(threadMti710);
    #endif
    thread thread_logger(threadWriter, 
                        (ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3),
                        (ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3));

    /* Join threads (program runs until killed) */
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread_iwr6843.join();
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread_mti710.join();
    #endif
    thread_logger.join();

#ifndef VALIDATE_PRINT
    csvRadar.close();
    csvImu.close();
#endif

    return 0;
}
