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

//— Globals ——————————————————————————————————————————————————————————————
IWR6843      radarSensor;
XsensMti710  imuSensor;

const int  NUM_THREADS = 2;
pthread_t  threads[NUM_THREADS];

ofstream   csvRadar("_outFiles/radar_output.csv");
ofstream   csvImu  ("_outFiles/imu_output.csv");

//— radar thread ———————————————————————————————————————————————————————
void* radar_thread(void*)
{
    if (!csvRadar.is_open()) {
        cerr << "[ERROR] Unable to open radar_output.csv\n";
        pthread_exit(nullptr);
    }
    csvRadar << "frame_id\t"
        << "point_id\t"
        <<"x\t"
        <<"y\t"
        <<"z\t"
        <<"doppler\t"
        <<"snr\t"
        <<"noise\n";

    csvImu << "frame_id\t"
        << "accel_x\taccel_y\taccel_z\t"
        << "free_accel_x\tfree_accel_y\tfree_accel_z\t"
        << "delta_v_x\tdelta_v_y\tdelta_v_z\t"
        << "delta_q_w\tdelta_q_x\tdelta_q_y\tdelta_q_z\t"
        << "rate_x\trate_y\trate_z\t"
        << "quat_w\tquat_x\tquat_y\tquat_z\t"
        << "mag_x\tmag_y\tmag_z\t"
        << "temperature\t"
        << "status_byte\t"
        << "packet_counter\t"
        << "time_fine\n";

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

        for (auto &f : frames) {
            auto hdr = f.getHeader();
            auto pd  = f.getTLVPayloadData();
            uint32_t fid = hdr.getFrameNumber();

            // log one IMU sample per radar frame
            MTiData mtiData = imuSensor.getXsensData();
            csvImu
              << fid << '\t'
              << mtiData.acceleration[0]     << '\t'
              << mtiData.acceleration[1]     << '\t'
              << mtiData.acceleration[2]     << '\t'
              << mtiData.free_acceleration[0]<< '\t'
              << mtiData.free_acceleration[1]<< '\t'
              << mtiData.free_acceleration[2]<< '\t'
              << mtiData.delta_v[0]          << '\t'
              << mtiData.delta_v[1]          << '\t'
              << mtiData.delta_v[2]          << '\t'
              << mtiData.delta_q[0]          << '\t'
              << mtiData.delta_q[1]          << '\t'
              << mtiData.delta_q[2]          << '\t'
              << mtiData.delta_q[3]          << '\t'
              << mtiData.rate_of_turn[0]     << '\t'
              << mtiData.rate_of_turn[1]     << '\t'
              << mtiData.rate_of_turn[2]     << '\t'
              << mtiData.quaternion[0]       << '\t'
              << mtiData.quaternion[1]       << '\t'
              << mtiData.quaternion[2]       << '\t'
              << mtiData.quaternion[3]       << '\t'
              << mtiData.magnetic[0]         << '\t'
              << mtiData.magnetic[1]         << '\t'
              << mtiData.magnetic[2]         << '\t'
              << mtiData.temperature         << '\t'
              << static_cast<int>(mtiData.status_byte) << '\t'
              << mtiData.packet_counter      << '\t'
              << mtiData.time_fine           << '\n';

            // then log each radar point
            for (size_t i = 0; i < pd.DetectedPoints_str.size(); ++i) {
                auto &pt    = pd.DetectedPoints_str[i];
                auto &side  = pd.SideInfoPoint_str;
                auto &noise = pd.NoiseProfilePoint_str;

                csvRadar
                  << fid        << '\t'
                  << (i + 1)    << '\t'
                  << pt.x_f     << '\t'
                  << pt.y_f     << '\t'
                  << pt.z_f     << '\t'
                  << pt.doppler_f << '\t'
                  << side.snr     << '\t'
                  << noise.noisePoint
                  << '\n';
            }
        }
    }

    csvRadar.close();
    csvImu.close();
    pthread_exit(nullptr);
}

//— imu thread ————————————————————————————————————————————————————————————
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

//— main ————————————————————————————————————————————————————————————————
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
    void* (*func[NUM_THREADS])(void*) = { radar_thread, imu_thread };
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
