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
#include <chrono>

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
/*
Macro to enable or disable radars
 1 - BOTH
 2 - LEFT ONLY
 3 - RIGHT ONLY
*/ 
#define ENABLE_RADAR_SIDE 1

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
    double timestamp;
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
static ofstream csvRadarLeft("_outFiles/radarLeft_calib.csv");
static ofstream csvRadarRight("_outFiles/radarRight_calib.csv");
static ofstream csvImu  ("_outFiles/imu_driveAround_calib.csv");
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
                    std::cout << "[DEBUG] Pushed LEFT batch of size: " << batch.size() << "\n";
                    for (const auto& pt : batch)
                    {
                        std::cout << "    [LEFT->Queue] Frame=" << pt.frameId
                                << " Idx=" << pt.pointId
                                << " x=" << pt.x
                                << " y=" << pt.y
                                << " z=" << pt.z
                                << " doppler=" << pt.doppler
                                << " snr=" << pt.snr
                                << " noise=" << pt.noise << "\n";
                    }
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
        rightRadarCount = radarRightSensor.poll();
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
        if (!radarRightSensor.copyDecodedFramesFromTop(rightRadarFrames, rightRadarCount, true, 100))
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
                lock_guard<mutex> lock(radarMutexRight);
                for (auto&& batch : rightFrameBatches)
                {
                    std::cout << "[DEBUG] Pushed RIGHT batch of size: " << batch.size() << "\n";
                    for (const auto& pt : batch)
                    {
                        std::cout << "    [RIGHT->Queue] Frame=" << pt.frameId
                                << " Idx=" << pt.pointId
                                << " x=" << pt.x
                                << " y=" << pt.y
                                << " z=" << pt.z
                                << " doppler=" << pt.doppler
                                << " snr=" << pt.snr
                                << " noise=" << pt.noise << "\n";
                    }
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
    const int packetSize = sizeof(PacketRadar);
    const int imuPacketSize = sizeof(PacketImu);

    while (true)
    {
        std::vector<ValidRadarPoint> radarPts;
        std::string source = "none";

        {
            std::unique_lock<std::mutex> leftLock(radarMutexLeft, std::defer_lock);
            std::unique_lock<std::mutex> rightLock(radarMutexRight, std::defer_lock);
            std::lock(leftLock, rightLock);

            #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 2
            if (!radarLeftQueue.empty())
            {
                radarPts = std::move(radarLeftQueue.front());
                radarLeftQueue.pop();
                source = "left";
            }
            #endif

            #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 3
            if (!radarRightQueue.empty())
            {
                radarPts = std::move(radarRightQueue.front());
                radarRightQueue.pop();
                source = "right";
            }
            #endif
        }

        if (radarPts.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const uint32_t fid = radarPts.front().frameId;

        // Send radar points over TCP
        if (enableRadar)
        {
            for (const auto& pt : radarPts)
            {
                PacketRadar packet;
                packet.frame_id = pt.frameId;
                packet.point_id = pt.pointId;
                packet.x = pt.x;
                packet.y = pt.y;
                packet.z = pt.z;
                packet.doppler = pt.doppler;
                packet.snr = pt.snr;
                packet.noise = pt.noise;
                packet.source = (source == "left") ? 0 : 1;

                if (send(clientSocket, reinterpret_cast<const char*>(&packet), packetSize, 0) < 0)
                {
                    std::cerr << "[ERROR] Failed to send radar packet\n";
                }
            }
        }

        // Send matching IMU data
        if (enableImu)
        {
            std::vector<MTiData> imuMatches;
            {
                std::lock_guard<std::mutex> lock(imuMutex);
                while (!imuQueue.empty())
                {
                    imuMatches.push_back(imuQueue.front());
                    imuQueue.pop();
                }
            }

            for (size_t i = 0UL; i < imuMatches.size(); ++i)
            {
                const MTiData& imu = imuMatches[i];
                PacketImu packet;

                packet.frame_id = fid;
                packet.imu_index = static_cast<uint32_t>(i + 1);

                // Quaternion
                packet.quat_w = imu.quaternion[0];
                packet.quat_x = imu.quaternion[1];
                packet.quat_y = imu.quaternion[2];
                packet.quat_z = imu.quaternion[3];

                // Acceleration
                packet.accel_x = imu.acceleration[0];
                packet.accel_y = imu.acceleration[1];
                packet.accel_z = imu.acceleration[2];

                // Free Acceleration
                packet.free_accel_x = imu.free_acceleration[0];
                packet.free_accel_y = imu.free_acceleration[1];
                packet.free_accel_z = imu.free_acceleration[2];

                // Delta V
                packet.delta_v_x = imu.delta_v[0];
                packet.delta_v_y = imu.delta_v[1];
                packet.delta_v_z = imu.delta_v[2];

                // Delta Q
                packet.delta_q_w = imu.delta_q[0];
                packet.delta_q_x = imu.delta_q[1];
                packet.delta_q_y = imu.delta_q[2];
                packet.delta_q_z = imu.delta_q[3];

                // Rate of turn
                packet.rate_x = imu.rate_of_turn[0];
                packet.rate_y = imu.rate_of_turn[1];
                packet.rate_z = imu.rate_of_turn[2];

                // Copy quaternion again as some formats require it
                packet.quat2_w = imu.quaternion[0];
                packet.quat2_x = imu.quaternion[1];
                packet.quat2_y = imu.quaternion[2];
                packet.quat2_z = imu.quaternion[3];

                // Magnetic field
                packet.mag_x = imu.magnetic[0];
                packet.mag_y = imu.magnetic[1];
                packet.mag_z = imu.magnetic[2];

                // Other status data
                packet.temperature = imu.temperature;
                packet.status_byte = imu.status_byte;
                packet.packet_counter = imu.packet_counter;
                packet.time_fine = imu.time_fine;

                // Add source (0=left, 1=right)
                packet.source = (source == "left") ? 0 : 1;

                if (send(clientSocket, reinterpret_cast<const char*>(&packet), imuPacketSize, 0) < 0)
                {
                    std::cerr << "[ERROR] Failed to send IMU packet\n";
                }
            }
        }
    }
}


/*=== MAIN ===*/
int main(void)
{
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
        #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 2
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
        #endif
        #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 3
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

    std::thread thread_iwr6843_left;
    std::thread thread_iwr6843_right;
    std::thread thread_mti710;
    std::thread thread_logger;

    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
        #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 2
            thread_iwr6843_left = std::thread(threadIwr6843Left);
        #endif
        #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 3
            thread_iwr6843_right = std::thread(threadIwr6843Right);
        #endif
    #endif

    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
        thread_mti710 = std::thread(threadMti710);
    #endif

    thread_logger = std::thread(threadWriter, 
                        (ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3),
                        (ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3));

    /* Join threads (program runs until killed) */
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
        #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 2
            thread_iwr6843_left.join();
        #endif
        #if ENABLE_RADAR_SIDE == 1 || ENABLE_RADAR_SIDE == 3
            thread_iwr6843_right.join();
        #endif
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