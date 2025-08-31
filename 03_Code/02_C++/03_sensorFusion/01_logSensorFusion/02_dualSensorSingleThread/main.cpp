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
#include <chrono>
#include <atomic>

#include "mmWave-IWR6843/radar_sensor/IWR6843.h"
#include "mmWave-IWR6843/radar_sensor/SensorData.h"
#include "MTi-G-710/xsens_mti710.hpp"

#include "misc/BoundedQueue.h"

using namespace std;

/*
Macro to enable or disable sensors
 1 - enable only IWR
 2 - enable only MTI
 3 - enable both
*/ 
#define ENABLE_SENSORS 3

const char CSV_TAB = ',';

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
    uint64_t timestamp;
};

/* Global synchronization objects */
static mutex                   radarMutexA;  /* Protects radarQueueA  */
static mutex                   radarMutexB;  /* Protects radarQueueB  */
static atomic<bool>            hasNewRadarA = false;
static atomic<bool>            hasNewRadarB = false;
static mutex                   imuMutex;    /* Protects imuQueue    */
static mutex                   writeMutex;  /* Protects writer sync */
static condition_variable      dataCV;      /* Signals data ready   */

/* Queues for inter-thread communication */
#define MAX_RADAR_QUEUE_SIZE 50
static BoundedQueue<vector<ValidRadarPoint>> radarQueueA;
static BoundedQueue<vector<ValidRadarPoint>> radarQueueB;
static BoundedQueue<MTiData>                 imuQueue;

/* Sensor objects */
static IWR6843     radarSensorA;
static IWR6843     radarSensorB;
static XsensMti710 imuSensor;

/* Global timer to obtain the program timestamp */
using Clock = std::chrono::steady_clock;
Clock::time_point programStart;

/* Output files */
static ofstream csvRadarA("_outFiles/radarA_2GHzConfig2.csv");
static ofstream csvRadarB("_outFiles/radarB_2GHzConfig2.csv");
static ofstream csvImu  ("_outFiles/imu_2GHzConfig2.csv");

static vector<vector<ValidRadarPoint>> extractValidRadarPoints(const vector<SensorData>& frames)
{
    vector<vector<ValidRadarPoint>> validFrames;

    for (const SensorData& frame : frames)
    {
        Frame_header frameHeader = frame.getHeader();
        uint32_t frameID = frameHeader.getFrameNumber();

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
                DetectedPoints& detectedPoints = payloadData.DetectedPoints_str[i];
                SideInfoPoint& sideInfo = payloadData.SideInfoPoint_str[i];
                uint64_t timestamp = elapsed_ms_since_start(programStart);

                validPoints.push_back(ValidRadarPoint{
                    frameID,
                    static_cast<uint32_t>(i + 1U),
                    detectedPoints.x_f,
                    detectedPoints.y_f,
                    detectedPoints.z_f,
                    detectedPoints.doppler_f,
                    sideInfo.snr,
                    sideInfo.noise,
                    timestamp
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

uint64_t elapsed_ms_since_start(std::chrono::steady_clock::time_point start) {
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
    );
}


/*=== threadIwr6843(): Radar acquisition & filtering ===*/
void threadIwr6843(void)
{
    int32_t RadarCountA = 0;
    int32_t RadarCountB = 0;
    for (;;)
    {
        // Poll information from both radars
        RadarCountA = radarSensorA.poll();
        RadarCountB = radarSensorB.poll();
        if ((RadarCountA < 0) || (RadarCountB < 0))
        {
            cerr << "[ERROR] Radar failed to poll\n";
            break;
        }
        if ((RadarCountA == 0) || (RadarCountB == 0))
        {
            // Wait for new data
            usleep(1000);
            continue;
        }

        vector<SensorData> RadarFramesA;
        vector<SensorData> RadarFramesB;
        if (!radarSensorA.copyDecodedFramesFromTop(RadarFramesA, RadarCountA, true, 100))
        {
            cerr << "[ERROR] Timeout copying radar A frames\n";
            continue;
        }
        if (!radarSensorB.copyDecodedFramesFromTop(RadarFramesB, RadarCountB, true, 100))
        {
            cerr << "[ERROR] Timeout copying radar B frames\n";
            continue;
        }

        const vector<vector<ValidRadarPoint>> FrameBatchesA = extractValidRadarPoints(RadarFramesA); 
        const vector<vector<ValidRadarPoint>> FrameBatchesB = extractValidRadarPoints(RadarFramesB); 

        /* For each decoded frame */
        // If the batch is not empty
        if(!FrameBatchesA.empty())
        {
            {
                lock_guard<mutex> lock(radarMutexA);
                for (auto&& batch : FrameBatchesA)
                {
                    std::cout << "[DEBUG] Frame batch of size: " << batch.size() << "\n";
                    for (const auto& pt : batch)
                    {
                        std::cout   << " Frame=" << pt.frameId
                                    << " Idx=" << pt.pointId
                                    << " x=" << pt.x
                                    << " y=" << pt.y
                                    << " z=" << pt.z
                                    << " doppler=" << pt.doppler
                                    << " snr=" << pt.snr
                                    << " noise=" << pt.noise << "\n"
                                    << " timestamp=" << pt.timestamp << "\n";
                    }
                    radarQueueA.push_drop_oldest(move(batch));
                    hasNewRadarA = true;
                }
            }
        }
        if(!FrameBatchesB.empty())
        {
            {
                lock_guard<mutex> lock(radarMutexB);
                for (auto&& batch : FrameBatchesB)
                {
                    std::cout << "[DEBUG] Frame batch of size: " << batch.size() << "\n";
                    for (const auto& pt : batch)
                    {
                        std::cout   << " Frame=" << pt.frameId
                                    << " Idx=" << pt.pointId
                                    << " x=" << pt.x
                                    << " y=" << pt.y
                                    << " z=" << pt.z
                                    << " doppler=" << pt.doppler
                                    << " snr=" << pt.snr
                                    << " noise=" << pt.noise << "\n"
                                    << " timestamp=" << pt.timestamp << "\n";
                    }
                    radarQueueB.push_drop_oldest(move(batch));
                    hasNewRadarB = true;
                }
            }
        }
        if(hasNewRadarA && hasNewRadarB)
        {
            dataCV.notify_one();
            hasNewRadarA = false;
            hasNewRadarB = false;
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
    if ((enableRadar && !csvRadarA.is_open()) || 
        (enableRadar && !csvRadarB.is_open()) ||
        (enableImu && !csvImu.is_open()))
    {
        cerr << "[ERROR] Output files not open!\n";
        return;
    }

    if (enableRadar)
    {
        csvRadarA << "frame_id,point_id,x,y,z,doppler,snr,noise\n";
        csvRadarB << "frame_id,point_id,x,y,z,doppler,snr,noise\n";
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

    for (;;)
    {
        /* === Wait for radar data === */
        unique_lock<mutex> writerLock(writeMutex);
        dataCV.wait(writerLock, [&]{
            return !radarQueueA.empty() && !radarQueueB.empty();
        });

        vector<ValidRadarPoint> radarPtsA;
        vector<ValidRadarPoint> radarPtsB;
        bool okA = false;
        bool okB = false;
        // Protect each queue individually
        {
            lock_guard<mutex> lockA(radarMutexA);
            okA = radarQueueA.try_pop(radarPtsA);
        }
        {
            lock_guard<mutex> lockB(radarMutexB);
            okB = radarQueueB.try_pop(radarPtsB);
        }
        if (!okA || !okB)
        {
            if(!okA) cerr << "[WARN] Failed to pop radarQueueA\n";
            if(!okB) cerr << "[WARN] Failed to pop radarQueueB\n";
        }
        writerLock.unlock();

        const size_t sizeA = radarPtsA.size();
        const size_t sizeB = radarPtsB.size();

        // Use min for conservative IMU data
        const size_t N = min(sizeA, sizeB);
        const uint32_t fid = radarPtsA.front().frameId;

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
            for (const auto& pt : radarPtsA)
            {
                csvRadarA << pt.frameId  << CSV_TAB
                         << pt.pointId  << CSV_TAB
                         << pt.x        << CSV_TAB
                         << pt.y        << CSV_TAB
                         << pt.z        << CSV_TAB
                         << pt.doppler  << CSV_TAB
                         << pt.snr      << CSV_TAB
                         << pt.noise    << CSV_TAB
                         << pt.timestamp<< "\n";
            }
            csvRadarA.flush();
            for (const auto& pt : radarPtsB)
            {
                csvRadarA << pt.frameId  << CSV_TAB
                         << pt.pointId  << CSV_TAB
                         << pt.x        << CSV_TAB
                         << pt.y        << CSV_TAB
                         << pt.z        << CSV_TAB
                         << pt.doppler  << CSV_TAB
                         << pt.snr      << CSV_TAB
                         << pt.noise    << CSV_TAB
                         << pt.timestamp<< "\n";
            }
            csvRadarB.flush();
        }

        /* === Write IMU data if enabled === */
        if (enableImu)
        {
            for (size_t i = 0UL; i < imuSamples.size(); ++i)
            {
                const MTiData& imu = imuSamples[i];
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
        }
    }
}

/*=== MAIN ===*/
int main(void)
{
    programStart = Clock::now(); // capture the global start time
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

    cout << "[INFO] Opening output files...\n";
    if ((!csvRadarA.is_open()) || (!csvImu.is_open()))
    {
        cerr << "[ERROR] Failed to open CSVs\n";
        return 1;
    }

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

    csvRadarA.close();
    csvImu.close();

    return 0;
}
