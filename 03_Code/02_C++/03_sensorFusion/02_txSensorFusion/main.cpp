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
// TCP server
#include <sstream>         // For std::ostringstream
#include <sys/socket.h>    // For socket(), bind(), listen(), accept(), send()
#include <netinet/in.h>    // For sockaddr_in, INADDR_ANY, AF_INET
#include <arpa/inet.h>     // For htons()

using namespace std;

/*
Macro to enable or disable sensors
 1 - enable only IWR
 2 - enable only MTI
 3 - enable both
*/ 
#define ENABLE_SENSORS 1

const char CSV_TAB = ',';
static int client_fd = -1;  // TCP connection socket

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
static mutex                   radarMutex;  /* Protects radarQueue  */
static mutex                   imuMutex;    /* Protects imuQueue    */
static mutex                   writeMutex;  /* Protects writer sync */
static condition_variable      dataCV;      /* Signals data ready   */

/* Queues for inter-thread communication */
static queue<vector<ValidRadarPoint>> radarQueue;
static queue<MTiData>                 imuQueue;

/* Sensor objects */
static IWR6843     radarSensor;
static XsensMti710 imuSensor;

/*=== threadIwr6843(): Radar acquisition & filtering ===*/
void threadIwr6843(void)
{
    for (;;)
    {
        int32_t cnt = radarSensor.poll();
        if (cnt < 0)
        {
            cerr << "[ERROR] radarSensor.poll() failed\n";
            break;
        }
        if (cnt == 0)
        {
            usleep(1000);
            continue;
        }

        vector<SensorData> frames;
        if (!radarSensor.copyDecodedFramesFromTop(frames, cnt, true, 100))
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
                        (min_diff      < 0.4F)                     // Range‐bin tolerance:  
                                                                   //    • 0.047 m/bin ⇒ ±2 bins ≈0.1 m  
                                                                   //    • bump to 0.2 m for ±4 bins if targets wander  
                                                                   //    • tighten (e.g. 0.1 m) if you know objects are point-like
                        &&                                          
                        (closest_power > 1800U));                  // Minimum peak power:  
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
                        lock_guard<mutex> lock(radarMutex);
                        radarQueue.push(move(validPoints));
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
    for (;;)
    {
        unique_lock<mutex> lk(writeMutex);

        dataCV.wait(lk, [&]{
            return ((enableRadar ? !radarQueue.empty() : true) &&
                    (enableImu   ? !imuQueue.empty()   : true));
        });

        vector<ValidRadarPoint> radarPts;
        if (enableRadar && !radarQueue.empty())
        {
            radarPts = move(radarQueue.front());
            radarQueue.pop();
        }
        lk.unlock();

        size_t imuNeeded = 0;
        if (enableRadar)
        {
            imuNeeded = radarPts.size() * 2UL;
        }

        vector<MTiData> imuSamples;
        if (enableImu && imuNeeded > 0UL)
        {
            unique_lock<mutex> lk2(writeMutex);
            dataCV.wait(lk2, [&]{
                return imuQueue.size() >= imuNeeded;
            });
            imuSamples.reserve(imuNeeded);
            {
                lock_guard<mutex> lock(imuMutex);
                for (size_t i = 0; i < imuNeeded; ++i)
                {
                    imuSamples.push_back(imuQueue.front());
                    imuQueue.pop();
                }
            }
        }

        // Send radar points
        if (enableRadar)
        {
            for (const auto& pt : radarPts)
            {
                std::ostringstream oss;
                oss << "[RADAR] frame=" << pt.frameId
                    << " pt="   << pt.pointId
                    << " x="    << pt.x
                    << " y="    << pt.y
                    << " z="    << pt.z
                    << " dop="  << pt.doppler
                    << " snr="  << pt.snr
                    << " noise="<< pt.noise
                    << "\n";

                if (client_fd >= 0)
                {
                    const std::string line = oss.str();
                    send(client_fd, line.c_str(), line.size(), 0);
                }
            }
        }

        // Send IMU samples
        if (enableImu)
        {
            for (size_t i = 0UL; i < imuSamples.size(); ++i)
            {
                const MTiData& imu = imuSamples[i];
                std::ostringstream oss;
                oss << "[IMU] frame=" << (enableRadar && !radarPts.empty() ? radarPts.front().frameId : 0)
                    << " idx=" << (i + 1UL)
                    << " quat=("
                    << imu.quaternion[0] << ","
                    << imu.quaternion[1] << ","
                    << imu.quaternion[2] << ","
                    << imu.quaternion[3] << ") "
                    << " accel=("
                    << imu.acceleration[0] << ","
                    << imu.acceleration[1] << ","
                    << imu.acceleration[2] << ") "
                    << " pkt=" << imu.packet_counter
                    << "\n";

                if (client_fd >= 0)
                {
                    const std::string line = oss.str();
                    send(client_fd, line.c_str(), line.size(), 0);
                }
            }
        }
    }
}


/*=== threadTcpServer(): synchronizes and sends data ===*/
void threadTcpServer()
{
    int server_fd;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0)
    {
        cerr << "[ERROR] TCP socket failed\n";
        return;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
    address.sin_port = htons(9000);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        cerr << "[ERROR] TCP bind failed\n";
        return;
    }

    if (listen(server_fd, 1) < 0)
    {
        cerr << "[ERROR] TCP listen failed\n";
        return;
    }

    cout << "[INFO] TCP Server waiting for client on port 9000...\n";
    client_fd = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
    if (client_fd < 0)
    {
        cerr << "[ERROR] TCP accept failed\n";
        return;
    }
    cout << "[INFO] TCP Client connected!\n";
}


/*=== MAIN ===*/
int main(void)
{
    #if ENABLE_SENSORS == 1 || ENABLE_SENSORS == 3
    cout << "[INFO] Initializing radar...\n";
    if (radarSensor.init("/dev/ttyUSB0",
                         "/dev/ttyUSB1",
                         "../01_logSensorFusion/mmWave-IWR6843/configs/"
                         "profile_azim60_elev30_calibrator.cfg"
                        ) != 1)
    {
        cerr << "[ERROR] radarSensor.init() failed\n";
        return 1;
    }
    #endif
    #if ENABLE_SENSORS == 2 || ENABLE_SENSORS == 3
    /* Step 1: initialize IMU */
    if ((imuSensor.findXsensDevice() != DEVICE_FOUND_SUCCESS) ||
        (imuSensor.openXsensPort()   != OPEN_PORT_SUCCESS))
    {
        cerr << "[ERROR] Unable to initialize IMU\n";
        return 1;
    }
    imuSensor.configure();
    #endif

    threadTcpServer();
    // When we get here, the client is connected
    if (client_fd < 0)
    {
        cerr << "[ERROR] No TCP client. Exiting.\n";
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

    return 0;
}
