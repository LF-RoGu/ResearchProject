// main.cpp

#include <iostream>
#include <fstream>
#include <vector>
#include <pthread.h>
#include <unistd.h>      // for read(), sleep()
#include <algorithm>     // for std::min
#include <cstdint>
#include <cstring>       // for strerror()

#include "mmWave-IWR6843/radar_sensor/IWR6843.h"
#include "mmWave-IWR6843/radar_sensor/SensorData.h"
#include "MTi-G-710/mti_utility.h"
#include "MTi-G-710/xsens_mti710.hpp"

using namespace std;

const char CSV_TAB = ',';

int g_noiseCounter = 0;

IWR6843      radarSensor;
XsensMti710  imuSensor;

const int  NUM_THREADS = 2;
pthread_t  threads[NUM_THREADS];

ofstream   csvRadar("_outFiles/radar_driveAround2.csv");
ofstream   csvImu  ("_outFiles/imu_driveAround2.csv");

void* sensor_thread(void*)
{
    if (!csvRadar.is_open()) {
        cerr << "[ERROR] Unable to open radar_output.csv\n";
        pthread_exit(nullptr);
    }
    if (!csvImu.is_open()) {
        cerr << "[ERROR] Unable to open radar_output.csv\n";
        pthread_exit(nullptr);
    }

    csvRadar 
        << "frame_id" << CSV_TAB 
        << "point_id" << CSV_TAB
        << "x" << CSV_TAB 
        << "y" << CSV_TAB 
        << "z" << CSV_TAB
        << "doppler" << CSV_TAB 
        << "snr" << CSV_TAB << "noise\n";

    csvImu << "frame_id" << CSV_TAB
        << "accel_x" << CSV_TAB 
        << "accel_y" << CSV_TAB 
        << "accel_z" << CSV_TAB
        << "free_accel_x" << CSV_TAB 
        << "free_accel_y" << CSV_TAB 
        << "free_accel_z" << CSV_TAB
        << "delta_v_x" << CSV_TAB 
        << "delta_v_y" << CSV_TAB 
        << "delta_v_z" << CSV_TAB
        << "delta_q_w" << CSV_TAB 
        << "delta_q_x" << CSV_TAB 
        << "delta_q_y" << CSV_TAB 
        << "delta_q_z" << CSV_TAB
        << "rate_x" << CSV_TAB 
        << "rate_y" << CSV_TAB 
        << "rate_z" << CSV_TAB
        << "quat_w" << CSV_TAB 
        << "quat_x" << CSV_TAB 
        << "quat_y" << CSV_TAB 
        << "quat_z" << CSV_TAB
        << "mag_x" << CSV_TAB 
        << "mag_y" << CSV_TAB 
        << "mag_z" << CSV_TAB
        << "temperature" << CSV_TAB 
        << "status_byte" << CSV_TAB 
        << "packet_counter" << CSV_TAB << "time_fine\n";

    while (true) {
        int cnt = radarSensor.poll();
        if (cnt < 0) {
            cerr << "[ERROR] radarSensor.poll() failed\n";
            break;
        }
        if (cnt == 0) {
            usleep(1000);
            continue;
        }

        vector<SensorData> frames;
        if (!radarSensor.copyDecodedFramesFromTop(frames, cnt, true, 100)) {
            cerr << "[ERROR] timed out copying radar frames\n";
            continue;
        }

        if (g_noiseCounter >= 10) 
        {
            for (auto &f : frames) 
            {
                auto hdr = f.getHeader();
                auto pd  = f.getTLVPayloadData();
                uint32_t fid = hdr.getFrameNumber();

                // log one IMU sample per radar frame
                MTiData mtiData = imuSensor.getXsensData();
                csvImu
                << fid << CSV_TAB
                << mtiData.acceleration[0]      << CSV_TAB
                << mtiData.acceleration[1]      << CSV_TAB
                << mtiData.acceleration[2]      << CSV_TAB
                << mtiData.free_acceleration[0] << CSV_TAB
                << mtiData.free_acceleration[1] << CSV_TAB
                << mtiData.free_acceleration[2] << CSV_TAB
                << mtiData.delta_v[0]           << CSV_TAB
                << mtiData.delta_v[1]           << CSV_TAB
                << mtiData.delta_v[2]           << CSV_TAB
                << mtiData.delta_q[0]           << CSV_TAB
                << mtiData.delta_q[1]           << CSV_TAB
                << mtiData.delta_q[2]           << CSV_TAB
                << mtiData.delta_q[3]           << CSV_TAB
                << mtiData.rate_of_turn[0]      << CSV_TAB
                << mtiData.rate_of_turn[1]      << CSV_TAB
                << mtiData.rate_of_turn[2]      << CSV_TAB
                << mtiData.quaternion[0]        << CSV_TAB
                << mtiData.quaternion[1]        << CSV_TAB
                << mtiData.quaternion[2]        << CSV_TAB
                << mtiData.quaternion[3]        << CSV_TAB
                << mtiData.magnetic[0]          << CSV_TAB
                << mtiData.magnetic[1]          << CSV_TAB
                << mtiData.magnetic[2]          << CSV_TAB
                << mtiData.temperature          << CSV_TAB
                << static_cast<int>(mtiData.status_byte) << CSV_TAB
                << mtiData.packet_counter       << CSV_TAB
                << mtiData.time_fine            << '\n';

                // then log each radar point
                for (size_t i = 0; i < pd.DetectedPoints_str.size(); ++i) {
                    auto &pt    = pd.DetectedPoints_str[i];
                    auto &side  = pd.SideInfoPoint_str;
                    auto &noise = pd.NoiseProfilePoint_str;

                    csvRadar
                    << fid         << CSV_TAB
                    << (i + 1)     << CSV_TAB
                    << pt.x_f      << CSV_TAB
                    << pt.y_f      << CSV_TAB
                    << pt.z_f      << CSV_TAB
                    << pt.doppler_f << CSV_TAB
                    << side.snr    << CSV_TAB
                    << noise.noisePoint
                    << '\n';
                }
            }
        }
        else
        {
            g_noiseCounter++;
        } 
    }

    csvRadar.close();
    csvImu.close();
    pthread_exit(nullptr);
}

void* imu_thread(void*)
{
    // find & open
    if (imuSensor.findXsensDevice() != DEVICE_FOUND_SUCCESS ||
        imuSensor.openXsensPort()   != OPEN_PORT_SUCCESS)
    {
        cerr << "[ERROR] Unable to initialize IMU\n";
        pthread_exit(nullptr);
    }

    // configure
    imuSensor.configure();

    // parser setup
    xsens_interface_t iface = XSENS_INTERFACE_RX(&XsensMti710::xsens_event_handler);
    uint8_t buf[256];

    // continuously drive parser
    while (true) {
        ssize_t n = read(imuSensor.getFd(), buf, sizeof(buf));
        if (n <= 0) {
            cerr << "[ERROR] IMU read error: " << strerror(errno) << "\n";
            break;
        }
        xsens_mti_parse_buffer(&iface, buf, n);
    }

    pthread_exit(nullptr);
}

int main()
{
    cout << "[INFO] Initializing radar...\n";
    if (radarSensor.init(
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "../01_logSensorFusion/mmWave-IWR6843/configs/profile_azim60_elev30_optimized.cfg"
        ) != 1)
    {
        cerr << "[ERROR] radarSensor.init() failed\n";
        return 1;
    }

    cout << "[INFO] Spawning threads...\n";
    void* (*func[NUM_THREADS])(void*) = { sensor_thread, imu_thread };
    for (int i = 0; i < NUM_THREADS; ++i) {
        if (pthread_create(&threads[i], nullptr, func[i], nullptr)) {
            cerr << "[ERROR] pthread_create #" << i << "\n";
            return 2;
        }
    }

    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], nullptr);
    }

    return 0;
}
