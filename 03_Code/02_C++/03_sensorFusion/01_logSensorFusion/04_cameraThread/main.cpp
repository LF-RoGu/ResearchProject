/* main.cpp
 *
 * MISRA C++:2008 Compliant Example
 * Demonstrates synchronized radar+IMU logging with 2×N IMU samples per N valid radar points.
 * - threadIwr6843A(): Reads mmWave frames, filters VALID points.
 * - threadMti710(): Reads Xsens IMU samples.
 * - threadWriter(): For each radar frame with N points, collects 2·N IMU samples and logs.
 */

#include <iostream>
#include <fstream>
#include <string>
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
#include <sstream>


#include "mmWave-IWR6843/radar_sensor/IWR6843.h"
#include "mmWave-IWR6843/radar_sensor/SensorData.h"
#include "MTi-G-710/xsens_mti710.hpp"

#include "misc/BoundedQueue.h"
#include "misc/tcpConnection.h"

#include "misc/CameraThread.h"

using namespace std;

#define RADAR_A_READY 0x1
#define RADAR_B_READY 0x2

/*
Macro to enable the TCP/UDP server for real-time data visualization
 0 - disable
 1 - enable
*/
#define ENABLE_REAL_TIME 0

/*
Macro to enable the camera visualization
 0 - disable
 1 - enable
*/
#define ENABLE_CAMERA 0

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
uint64_t frameIdSensorA = 0;
uint64_t frameIdSensorB = 0;
uint64_t frameIdImu = 0;

/* Global synchronization objects */
static mutex                   radarMutexA;  /* Protects radarQueueA  */
static mutex                   radarMutexB;  /* Protects radarQueueB  */
static mutex                   decodeMutexA; /* Protects decode radar A */
static mutex                   decodeMutexB; /* Protects decode radar B */
static atomic<bool>            hasNewRadarA = false;
static atomic<bool>            hasNewRadarB = false;
static atomic<bool>            hasNewImu    = false;
static mutex                   imuMutex;    /* Protects imuQueue    */
static mutex                   writeMutex;  /* Protects writer sync */
static condition_variable      dataCV;      /* Signals data ready   */
static mutex                   csvMutexRadarA;  /* Protects csvRadarA   */
static mutex                   csvMutexRadarB;  /* Protects csvRadarB   */
static mutex                   csvMutexIMU;     /* Protects csvRadarB   */
static mutex                   dataAccessMutex; /* Protects decode radar */

/* Queues for inter-thread communication */
#define MAX_BOUNDED_QUEUE_SIZE 4000
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
const string fileSuffix = "hallway1";  // << Change this only once
static ofstream csvRadarA("_outFiles/radarA_" + fileSuffix + ".csv");
static ofstream csvRadarB("_outFiles/radarB_" + fileSuffix + ".csv");
static ofstream csvImu   ("_outFiles/imu_" + fileSuffix + ".csv");


uint64_t elapsed_ms_since_start(std::chrono::steady_clock::time_point start) {
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
    );
}

static vector<vector<ValidRadarPoint>> extractValidRadarPoints(const vector<SensorData>& frames, const uint64_t localFrameId)
{
    vector<vector<ValidRadarPoint>> validFrames;

    for (const SensorData& frame : frames)
    {
        Frame_header frameHeader = frame.getHeader();
        //uint32_t frameID = frameHeader.getFrameNumber();
        uint32_t frameID = localFrameId;

        for (TLVPayloadData& payloadData : frame.getTLVPayloadData())
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
            for (size_t i = 0UL; i < payloadData.DetectedPoints_str.size(); i++)
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

// Helper: Element-wise average of MTiData
MTiData averageImuSamples(const std::vector<MTiData>& samples)
{
    MTiData avg = {};
    const size_t N = samples.size();

    for (const MTiData& s : samples)
    {
        for (size_t i = 0; i < 4; ++i) {
            avg.quaternion[i] += s.quaternion[i];
            avg.delta_q[i]    += s.delta_q[i];
        }
        for (size_t i = 0; i < 3; ++i) {
            avg.acceleration[i]      += s.acceleration[i];
            avg.free_acceleration[i] += s.free_acceleration[i];
            avg.delta_v[i]           += s.delta_v[i];
            avg.rate_of_turn[i]      += s.rate_of_turn[i];
            avg.magnetic[i]          += s.magnetic[i];
        }
        avg.temperature    += s.temperature;
        avg.status_byte    |= s.status_byte;  // OR status bits
        avg.packet_counter += s.packet_counter;
        avg.time_fine      += s.time_fine;
    }

    // Normalize
    for (size_t i = 0; i < 4; ++i) {
        avg.quaternion[i] /= N;
        avg.delta_q[i]    /= N;
    }
    for (size_t i = 0; i < 3; ++i) {
        avg.acceleration[i]      /= N;
        avg.free_acceleration[i] /= N;
        avg.delta_v[i]           /= N;
        avg.rate_of_turn[i]      /= N;
        avg.magnetic[i]          /= N;
    }
    avg.temperature    /= N;
    avg.packet_counter /= N;
    avg.time_fine      /= N;

    return avg;
}

/*=== threadIwr6843A(): Radar acquisition & filtering ===*/
void threadIwr6843A(void)
{
    int32_t RadarCountA = 0;

    for (;;)
    {
        // === Lock decode mutex while polling radar A ===
        unique_lock<mutex> dataLock(decodeMutexA);

        // Poll new data from radar A
        RadarCountA = radarSensorA.poll();
        if (RadarCountA < 0)
        {
            dataLock.unlock();
            cerr << "[ERROR] Radar A failed to poll\n";
            break;
        }

        if (RadarCountA == 0)
        {
            dataLock.unlock();
            // No new data yet — sleep briefly
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }

        // === Try to copy the polled radar frames ===
        vector<SensorData> RadarFramesA;
        if (!radarSensorA.copyDecodedFramesFromTop(RadarFramesA, RadarCountA, true, 100))
        {
            dataLock.unlock();
            cerr << "[ERROR] Timeout copying radar A frames\n";
            continue;
        }

        // Done with radar poll and decode
        dataLock.unlock();

        // Extract valid radar points into per-frame batches
        frameIdSensorA++;
        vector<vector<ValidRadarPoint>> FrameBatchesA = extractValidRadarPoints(RadarFramesA, frameIdSensorA); 

        // === If frames were successfully decoded and extracted ===
        if (!FrameBatchesA.empty())
        {
            // Lock queue access to protect shared radarQueueA
            lock_guard<mutex> lock(radarMutexA);
            for (auto&& batch : FrameBatchesA)
            {
                lock_guard<mutex> lock(csvMutexRadarA);
                {
                    std::cout << "[SENSOR A] Frame batch of size: " << batch.size() << "\n";
                    for (const auto& pt : batch)
                    {
                        
                        // Write radar point to CSV
                        csvRadarA   << pt.frameId   << CSV_TAB
                                    << pt.pointId   << CSV_TAB
                                    << pt.x         << CSV_TAB
                                    << pt.y         << CSV_TAB
                                    << pt.z         << CSV_TAB
                                    << pt.doppler   << CSV_TAB
                                    << pt.snr       << CSV_TAB
                                    << pt.noise     << CSV_TAB
                                    << pt.timestamp << "\n";
                        
                        /*
                        std::cout   << " Frame=" << pt.frameId
                                    << " Idx=" << pt.pointId
                                    << " x=" << pt.x
                                    << " y=" << pt.y
                                    << " z=" << pt.z
                                    << " doppler=" << pt.doppler
                                    << " snr=" << pt.snr
                                    << " noise=" << pt.noise << "\n"
                                    << " timestamp=" << pt.timestamp << "\n";
                        */
                    }
                    csvRadarA.flush();
                    #if ENABLE_REAL_TIME
                    {
                        radarQueueA.push_drop_oldest(std::move(batch));
                    }
                    #endif
                    hasNewRadarA.store(true);
                }
            }
        }

        // Notify the consumer that new radar data is available
        if (hasNewRadarA)
        {
            dataCV.notify_one();
        }
    }
}

void threadIwr6843B(void)
{
    int32_t RadarCountB = 0;
    for (;;)
    {
        unique_lock<mutex> dataLock(decodeMutexB);
        // Poll information from both radars
        RadarCountB = radarSensorB.poll();
        if ((RadarCountB < 0))
        {
            dataLock.unlock();
            cerr << "[ERROR] Radar failed to poll\n";
            break;
        }
        if ((RadarCountB == 0))
        {
            dataLock.unlock();
            // Wait for new data
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }

        vector<SensorData> RadarFramesB;
        if (!radarSensorB.copyDecodedFramesFromTop(RadarFramesB, RadarCountB, true, 100))
        {
            dataLock.unlock();
            cerr << "[ERROR] Timeout copying radar B frames\n";
            continue;
        }

        dataLock.unlock();
        frameIdSensorB++;
        vector<vector<ValidRadarPoint>> FrameBatchesB = extractValidRadarPoints(RadarFramesB, frameIdSensorB); 

        /* For each decoded frame */
        // If the batch is not empty
        if(!FrameBatchesB.empty())
        {
            {
                // Lock queue access to protect shared radarQueueA
                lock_guard<mutex> lock(radarMutexB);
                for (auto&& batch : FrameBatchesB)
                {
                    lock_guard<mutex> lock(csvMutexRadarB);
                    {
                        std::cout << "[SENSOR B] Frame batch of size: " << batch.size() << "\n";
                        for (const auto& pt : batch)
                        {
                            csvRadarB   << pt.frameId   << CSV_TAB
                                        << pt.pointId   << CSV_TAB
                                        << pt.x         << CSV_TAB
                                        << pt.y         << CSV_TAB
                                        << pt.z         << CSV_TAB
                                        << pt.doppler   << CSV_TAB
                                        << pt.snr       << CSV_TAB
                                        << pt.noise     << CSV_TAB
                                        << pt.timestamp << "\n";
                            
                            /*
                            std::cout   << " Frame=" << pt.frameId
                                        << " Idx=" << pt.pointId
                                        << " x=" << pt.x
                                        << " y=" << pt.y
                                        << " z=" << pt.z
                                        << " doppler=" << pt.doppler
                                        << " snr=" << pt.snr
                                        << " noise=" << pt.noise << "\n"
                                        << " timestamp=" << pt.timestamp << "\n";
                            */
                        }
                        csvRadarB.flush();
                        #if ENABLE_REAL_TIME
                        {
                            radarQueueB.push_drop_oldest(move(batch));
                        }
                        #endif
                        hasNewRadarB.store(true);
                    }
                }   
            }
        }
        if(hasNewRadarB)
        {
            dataCV.notify_one();
        }
    }
}

/*=== threadMti710(): IMU acquisition ===*/
void threadMti710(void)
{
    xsens_interface_t iface = XSENS_INTERFACE_RX(&XsensMti710::xsens_event_handler);
    uint8_t buf[256];

    vector<MTiData> imuSamples;
    imuSamples.reserve(4);
    for (;;)
    {
        /* Wait for radar data to be available */
        {
            unique_lock<mutex> lock(writeMutex);  // assuming same mutex used by radar threads
            dataCV.wait(lock, [&] {
                return hasNewRadarA.load() && hasNewRadarB.load();
            });
            hasNewRadarA.store(false);
            hasNewRadarB.store(false);
        }
        imuSamples.clear();

        while (imuSamples.size() < 4)
        {
            const ssize_t n = read(imuSensor.getFd(), buf, sizeof(buf));
            if (n <= 0)
            {
                cerr << "[ERROR] IMU read error\n";
                break;
            }

            xsens_mti_parse_buffer(&iface, buf, static_cast<size_t>(n));
            MTiData data = imuSensor.getXsensData();
            imuSamples.push_back(data);
        }

        lock_guard<mutex> lock(csvMutexIMU);
        {
            frameIdImu++;
            for(uint8_t i = 0; i < imuSamples.size(); i++)
            {
                const MTiData& imu = imuSamples[i];
                csvImu << frameIdImu                << CSV_TAB
                       << (i + 1UL)                 << CSV_TAB
                       << imu.quaternion[0]         << CSV_TAB
                       << imu.quaternion[1]         << CSV_TAB
                       << imu.quaternion[2]         << CSV_TAB
                       << imu.quaternion[3]         << CSV_TAB
                       << imu.acceleration[0]       << CSV_TAB
                       << imu.acceleration[1]       << CSV_TAB
                       << imu.acceleration[2]       << CSV_TAB
                       << imu.free_acceleration[0]  << CSV_TAB
                       << imu.free_acceleration[1]  << CSV_TAB
                       << imu.free_acceleration[2]  << CSV_TAB
                       << imu.delta_v[0]            << CSV_TAB
                       << imu.delta_v[1]            << CSV_TAB
                       << imu.delta_v[2]            << CSV_TAB
                       << imu.delta_q[0]            << CSV_TAB
                       << imu.delta_q[1]            << CSV_TAB
                       << imu.delta_q[2]            << CSV_TAB
                       << imu.delta_q[3]            << CSV_TAB
                       << imu.rate_of_turn[0]       << CSV_TAB
                       << imu.rate_of_turn[1]       << CSV_TAB
                       << imu.rate_of_turn[2]       << CSV_TAB
                       << imu.quaternion[0]         << CSV_TAB
                       << imu.quaternion[1]         << CSV_TAB
                       << imu.quaternion[2]         << CSV_TAB
                       << imu.quaternion[3]         << CSV_TAB
                       << imu.magnetic[0]           << CSV_TAB
                       << imu.magnetic[1]           << CSV_TAB
                       << imu.magnetic[2]           << CSV_TAB
                       << imu.temperature           << CSV_TAB
                       << int(imu.status_byte)      << CSV_TAB
                       << imu.packet_counter        << CSV_TAB
                       << imu.time_fine             << "\n";
            }
            csvImu.flush();
            hasNewImu.store(true);
            std::cout   << "[SYNC] IMU Frame = " << frameIdImu
                        << ", RadarA Frame = " << frameIdSensorA
                        << ", RadarB Frame = " << frameIdSensorB << "\n";

        }
        #if ENABLE_REAL_TIME
        {
            const MTiData avg = averageImuSamples(imuSamples);
            {
                lock_guard<mutex> lock(imuMutex);
                if(imuQueue.size() >= MAX_BOUNDED_QUEUE_SIZE)
                {
                    std::cerr << "[WARN] IMU queue full (size = " << imuQueue.size() << "), dropping data\n";
                }
                else
                {
                    imuQueue.push(avg);
                }
            }
        }
        #endif

        if(hasNewImu)
        {
            dataCV.notify_one();
        }
    }
}

#if ENABLE_REAL_TIME
void threadRealTimeTransmit(int tcpSocketFd) {
    std::cout << "[REALTIME] TCP transmitter thread started\n";

    MTiData latestImu;
    std::vector<ValidRadarPoint> latestRadarA;
    std::vector<ValidRadarPoint> latestRadarB;

    for (;;) {
        bool updated = false;

        MTiData imuData;
        std::vector<ValidRadarPoint> radarDataA;
        std::vector<ValidRadarPoint> radarDataB;

        if (imuQueue.try_pop(imuData)) {
            latestImu = imuData;
            updated = true;
            std::cout << "[TCP] Got IMU data\n";
        }

        if (radarQueueA.try_pop(radarDataA)) {
            latestRadarA = std::move(radarDataA);
            updated = true;
            std::cout << "[TCP] Got Radar A data\n";
        }

        if (radarQueueB.try_pop(radarDataB)) {
            latestRadarB = std::move(radarDataB);
            updated = true;
            std::cout << "[TCP] Got Radar B data\n";
        }

        if (updated) {
            std::ostringstream ss;
            ss << "IMU," << latestImu.quaternion[0] << "," << latestImu.quaternion[1]
               << "," << latestImu.quaternion[2] << "," << latestImu.quaternion[3] << "\n";

            ss << "RADAR_A," << latestRadarA.size() << "\n";
            for (const auto& pt : latestRadarA) {
                ss << pt.frameId << "," << pt.x << "," << pt.y << "," << pt.z << "," << pt.doppler << "\n";
            }

            ss << "RADAR_B," << latestRadarB.size() << "\n";
            for (const auto& pt : latestRadarB) {
                ss << pt.frameId << "," << pt.x << "," << pt.y << "," << pt.z << "," << pt.doppler << "\n";
            }

            std::string packet = ss.str();
            ssize_t sent = send(tcpSocketFd, packet.c_str(), packet.size(), 0);
            if (sent < 0) {
                perror("[REALTIME] send");
                break;
            }

            std::cout << "[TCP] Packet sent. Size: " << sent << " bytes\n";
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    close(tcpSocketFd);
}

#endif


void flushSerialPort(const char* devicePath);
/*=== MAIN ===*/
int main(void)
{
    #if ENABLE_CAMERA == 1
    CameraThread camera("/dev/video0", "camera_output", 10);
    #endif
    #if ENABLE_REAL_TIME
        int socketFd = -1;
        socketFd = setupTcpSocket("0.0.0.0", 8888);  // Or your desired bind IP
        if (socketFd < 0) {
            std::cerr << "[ERROR] Could not initialize TCP socket.\n";
            return 1;
        }
        std::cout << "[INFO] TCP socket opened on port 8888\n";
    #endif
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    cout << "[INFO] Initializing radarA...\n";
    // Left sensor
    if (radarSensorA.init("/dev/sensorA_config",
                         "/dev/sensorA_data",
                         "/home/luis/Desktop/ResearchProject/03_Code/02_C++/03_sensorFusion/01_logSensorFusion/03_dualSensorSeparateThread/mmWave-IWR6843/configs/"
                         "left_profile_azim60_elev30_calibrator.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] radarSensorA.init() failed\n";
        return 1;
    }
    cout << "[INFO] Initializing radarB...\n";
    // Right sensor
    if (radarSensorB.init("/dev/sensorB_config",
                         "/dev/sensorB_data",
                         "/home/luis/Desktop/ResearchProject/03_Code/02_C++/03_sensorFusion/01_logSensorFusion/03_dualSensorSeparateThread/mmWave-IWR6843/configs/"
                         "right_profile_azim60_elev30_calibrator.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] radarSensorB.init() failed\n";
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
    if ((!csvRadarA.is_open()) || (!csvRadarB.is_open()) || (!csvImu.is_open()))
    {
        cerr << "[ERROR] Failed to open CSVs\n";
        return 1;
    }

    if (ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3)
    {
        csvRadarA << "frame_id,point_id,x,y,z,doppler,snr,noise,timestamp\n";
        csvRadarB << "frame_id,point_id,x,y,z,doppler,snr,noise,timestamp\n";
    }
    if (ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3)
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

    this_thread::sleep_for(chrono::milliseconds(1000));

    flushSerialPort("/dev/ttyUSB1");  // Clear input buffer of /dev/ttyUSB1
    flushSerialPort("/dev/ttyUSB3"); // Clear input buffer of /dev/ttyUSB3

    programStart = Clock::now(); // capture the global start time

    #if ENABLE_CAMERA == 1
    camera.start();  // Start camera thread (~10 FPS image saving)
    #endif

    cout << "[INFO] Spawning threads...\n";
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread thread_iwr6843A(threadIwr6843A);
    thread thread_iwr6843B(threadIwr6843B);
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread thread_mti710(threadMti710);
    #endif
    #if ENABLE_REAL_TIME
    thread tcpThread(threadRealTimeTransmit, socketFd);
    #endif

    /* Join threads (program runs until killed) */
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    thread_iwr6843A.join();
    thread_iwr6843B.join();
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    thread_mti710.join();
    #endif
    #if ENABLE_REAL_TIME
    tcpThread.join();
    closeTcpSocket(socketFd);
    std::cout << "[INFO] TCP socket closed\n";
    #endif
    #if ENABLE_CAMERA == 1
    camera.stop();  // Stop camera thread (~10 FPS image saving)
    #endif

    csvRadarA.close();
    csvImu.close();

    return 0;
}

void flushSerialPort(const char* devicePath)
{
    int fd = open(devicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "[ERROR] Could not open " << devicePath << " for flushing\n";
        return;
    }

    if (tcflush(fd, TCIFLUSH) == -1) {
        std::cerr << "[ERROR] Failed to flush RX buffer for " << devicePath << "\n";
    } else {
        std::cout << "[INFO] Flushed RX buffer for " << devicePath << "\n";
    }

    close(fd);
}