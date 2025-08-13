/* main.cpp
 *
 * MISRA C++:2008 Compliant Example
 * Demonstrates synchronized radar+IMU logging with 2×N IMU samples per N valid radar points.
 * - threadIwr6843Left(): Reads mmWave frames, filters VALID points.
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
static mutex                   radarMutexLeft;  /* Protects radarQueue  */
static mutex                   radarMutexRight; /* Protects radarQueue  */
static mutex                   imuMutex;        /* Protects imuQueue    */
static mutex                   writeMutex;      /* Protects writer sync */
static condition_variable      dataCV;          /* Signals data ready   */

/* Queues for inter-thread communication */
static queue<vector<ValidRadarPoint>> radarLeftQueue;
static queue<vector<ValidRadarPoint>> radarRightQueue;
static queue<MTiData>                 imuQueue;

/* Sensor objects */
static IWR6843     radarLeftSensor;
static IWR6843     radarRightSensor;
static XsensMti710 imuSensor;

#ifndef VALIDATE_PRINT
static ofstream csvRadarLeft("_outFiles/radarLeft_hallway_3.csv");
static ofstream csvRadarRight("_outFiles/radarRight_hallway_3.csv");
static ofstream csvImu  ("_outFiles/imu_hallway_3.csv");
#endif

const int UPDATE_POWER = 2000U; /* Minimum peak power for VALID radar points */
const float BIN_TOLERANCE = 0.2;

static vector<vector<ValidRadarPoint>> extractValidRadarPoints(const vector<SensorData>& frames)
{
    vector<vector<ValidRadarPoint>> validFrames;

    for (const SensorData& frame : frames)
    {
        const Frame_header hdr = frame.getHeader();
        const uint32_t fid = hdr.getFrameNumber();

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

            vector<ValidRadarPoint> validPoints;
            for (size_t i = 0UL; i < pd.DetectedPoints_str.size(); ++i)
            {
                const DetectedPoints& dp = pd.DetectedPoints_str[i];
                const float pt_range = sqrtf(dp.x_f * dp.x_f +
                                             dp.y_f * dp.y_f +
                                             dp.z_f * dp.z_f);

                float closest_range = -1.0F;
                uint16_t closest_power = 0U;
                float min_diff = numeric_limits<float>::max();
                for (const RangeProfilePoint& rp : pd.RangeProfilePoint_str)
                {
                    const float diff = fabsf(pt_range - rp.range_f);
                    if (diff < min_diff)
                    {
                        min_diff = diff;
                        closest_range = rp.range_f;
                        closest_power = rp.power_u16;
                    }
                }

                const bool is_valid =
                    ((closest_range >= 0.0F) &&
                     (min_diff < BIN_TOLERANCE) &&
                     (closest_power > UPDATE_POWER));

                if (is_valid && (i < pd.SideInfoPoint_str.size()))
                {
                    const SideInfoPoint& si = pd.SideInfoPoint_str[i];
                    validPoints.push_back(ValidRadarPoint{
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

            if (!validPoints.empty())
            {
                validFrames.push_back(std::move(validPoints));
            }
        }
    }

    return validFrames;
}

/*=== threadIwr6843Left(): Radar acquisition & filtering ===*/
void threadIwr6843Left(void)
{
    int32_t leftRadarCount = 0;
    for (;;)
    {
        // Poll information from both radars
        leftRadarCount = radarLeftSensor.poll();
        if (leftRadarCount < 0)
        {
            cerr << "[ERROR] Both radars failed to poll\n";
            break;
        }
        if (leftRadarCount == 0)
        {
            // Wait for new data
            usleep(1000);
            continue;
        }

        vector<SensorData> leftRadarFrames;
        if (!radarLeftSensor.copyDecodedFramesFromTop(leftRadarFrames, leftRadarCount, true, 100))
        {
            cerr << "[ERROR] Timeout copying radar frames\n";
            continue;
        }

        const vector<vector<ValidRadarPoint>> leftFrameBatches = extractValidRadarPoints(leftRadarFrames); 

        /* For each decoded frame */
        // If the batch is not empty
        if(!leftFrameBatches.empty())
        {
            {
                lock_guard<mutex> lock(radarMutexLeft);
                for (auto&& batch : leftFrameBatches)
                {
                    radarLeftQueue.push(std::move(batch));
                }
            }
            dataCV.notify_one();
        }
    }
}

/*=== threadIwr6843Right(): Radar acquisition & filtering ===*/
void threadIwr6843Right(void)
{
    int32_t rightRadarCount = 0;
    for (;;)
    {
        // Poll information from both radars
        rightRadarCount = radarLeftSensor.poll();
        if (rightRadarCount < 0)
        {
            cerr << "[ERROR] Both radars failed to poll\n";
            break;
        }
        if (rightRadarCount == 0)
        {
            // Wait for new data
            usleep(1000);
            continue;
        }

        vector<SensorData> rightRadarFrames;
        if (!radarLeftSensor.copyDecodedFramesFromTop(rightRadarFrames, rightRadarCount, true, 100))
        {
            cerr << "[ERROR] Timeout copying radar frames\n";
            continue;
        }

        const vector<vector<ValidRadarPoint>> rightFrameBatches = extractValidRadarPoints(rightRadarFrames); 

        /* For each decoded frame */
        // If the batch is not empty
        if(!rightFrameBatches.empty())
        {
            {
                lock_guard<mutex> lock(radarMutexLeft);
                for (auto&& batch : rightFrameBatches)
                {
                    radarRightQueue.push(std::move(batch));
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
    if ((enableRadar && !csvRadarLeft.is_open()) || 
        (enableRadar && !csvRadarRight.is_open()) ||
        (enableImu && !csvImu.is_open()))
    {
        cerr << "[ERROR] Output files not open!\n";
        return;
    }

    if (enableRadar)
    {
        csvRadarLeft << "frame_id,point_id,x,y,z,doppler,snr,noise\n";
        csvRadarRight << "frame_id,point_id,x,y,z,doppler,snr,noise\n";
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
        unique_lock<mutex> radarLockLeft(radarMutexLeft);
        unique_lock<mutex> radarLockRight(radarMutexRight);
        dataCV.wait(radarLockLeft, [&]{
            return !radarLeftQueue.empty();
        });
        dataCV.wait(radarLockRight, [&]{
            return !radarRightQueue.empty();
        });

        vector<ValidRadarPoint> leftRadarPts    = move(radarLeftQueue.front());
        vector<ValidRadarPoint> rightRadarPts   = move(radarRightQueue.front());
        radarLeftQueue.pop();
        radarRightQueue.pop();
        radarLockLeft.unlock();
        radarLockRight.unlock();

        size_t N;
        uint32_t fid;
        if(leftRadarPts.size() > rightRadarPts.size())
        {
            N = leftRadarPts.size();
            fid = leftRadarPts.front().frameId;
        }
        else
        {
            N = rightRadarPts.size();
            fid = rightRadarPts.front().frameId;
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
            for (const auto& pt : leftRadarPts)
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
            for (const auto& pt : leftRadarPts)
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
#ifdef VALIDATE_PRINT
            for (const auto& pt : rightRadarPts)
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
            for (const auto& pt : rightRadarPts)
            {
                csvRadarRight << pt.frameId  << CSV_TAB
                         << pt.pointId  << CSV_TAB
                         << pt.x        << CSV_TAB
                         << pt.y        << CSV_TAB
                         << pt.z        << CSV_TAB
                         << pt.doppler  << CSV_TAB
                         << pt.snr      << CSV_TAB
                         << pt.noise    << "\n";
            }
            csvRadarRight.flush();
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
    cout << "[INFO] Initializing LEFT radar...\n";
    if (radarLeftSensor.init(
                         "/dev/ttyUSB0",
                         "/dev/ttyUSB1",
                         "../01_logSensorFusion/mmWave-IWR6843/configs/"
                         "left_profile_azim60_elev30_calibrator.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] radarLeftSensor.init() failed\n";
        return 1;
    }
    cout << "[INFO] Initializing RIGHT radar...\n";
    if (radarRightSensor.init(
                         "/dev/ttyUSB2",
                         "/dev/ttyUSB3",
                         "../01_logSensorFusion/mmWave-IWR6843/configs/"
                         "right_profile_azim60_elev30_calibrator.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] radarRightSensor.init() failed\n";
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
    if ((!csvRadarLeft.is_open()) || (!csvRadarRight.is_open()) || (!csvImu.is_open()))
    {
        cerr << "[ERROR] Failed to open CSVs\n";
        return 1;
    }
#endif

    cout << "[INFO] Spawning threads...\n";
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread thread_iwr6843_left(threadIwr6843Left);
    thread thread_iwr6843_right(threadIwr6843Right);
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread thread_mti710(threadMti710);
    #endif
    thread thread_logger(threadWriter, 
                        (ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3),
                        (ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3));

    /* Join threads (program runs until killed) */
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread_iwr6843_left.join();
    thread_iwr6843_right.join();
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread_mti710.join();
    #endif
    thread_logger.join();

#ifndef VALIDATE_PRINT
    csvRadarLeft.close();
    csvRadarRight.close();
    csvImu.close();
#endif

    return 0;
}
