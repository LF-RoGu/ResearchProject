#include "Radar.h"
#include <unistd.h>    // for sleep()
#include <algorithm>   // for std::min

IWR6843    sensor;
const int  NUM_THREADS = 3;
pthread_t  threads[NUM_THREADS];

void* sensor_thread(void* /*arg*/)
{
    while (true)
    {
        int newCount = sensor.poll();
        if (newCount < 0) {
            cerr << "[ERROR] sensor.poll() failed\n";
            break;
        }
        if (newCount == 0) {
            // no data yet
            continue;
        }

        vector<SensorData> frames;
        if (!sensor.copyDecodedFramesFromTop(frames, newCount, true, 100)) {
            cerr << "[ERROR] timed out copying frames\n";
            continue;
        }

        for (auto &frame : frames)
        {
            auto hdr = frame.getHeader();
            auto pd  = frame.getTLVPayloadData();

            if (pd.SideInfoPoint_str.size() != pd.DetectedPoints_str.size()) {
                std::cerr << "[ERROR] Mismatch: Detected=" << pd.DetectedPoints_str.size()
                        << " vs SideInfo=" << pd.SideInfoPoint_str.size() << std::endl;
                return 0;
            }


            uint32_t frame_id = hdr.getFrameNumber();
            size_t   pts      = pd.DetectedPoints_str.size();

            for (size_t i = 0; i < pts; ++i) {
                auto& pt   = pd.DetectedPoints_str[i];
                auto& side = pd.SideInfoPoint_str[i];

                float snr_dB   = side.snr * 0.1f;
                float noise_dB = side.noise * 0.1f;

                cout << frame_id << '\t'
                    << (i + 1)  << '\t'
                    << pt.x_f   << '\t'
                    << pt.y_f   << '\t'
                    << pt.z_f   << '\t'
                    << pt.doppler_f << '\t'
                    << snr_dB   << '\t'
                    << noise_dB << '\n';
            }
        }
    }

    pthread_exit(nullptr);
}

void* controller_thread(void* /*arg*/)
{
    auto tid = pthread_self();
    cout << "[CTRL] thread " << tid << " running...\n";
    sleep(10);
    pthread_exit(nullptr);
}

void* actuator_thread(void* /*arg*/)
{
    auto tid = pthread_self();
    cout << "[ACT ] thread " << tid << " running...\n";
    sleep(10);
    pthread_exit(nullptr);
}

int main()
{
    // initialize sensor
    cout << "[INFO] Initializing sensor...\n";
    int initResult = sensor.init(
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
        "./configs/profile_azim60_elev30_optimized.cfg"
    );
    if (initResult != 1) {
        cerr << "[ERROR] sensor.init() returned " << initResult << ":\n";
        switch(initResult) {
            case 0:  cerr << "  • Failed to open config port\n"; break;
            case -1: cerr << "  • Failed to configure config port\n"; break;
            case -2: cerr << "  • Failed to open data port\n"; break;
            case -3: cerr << "  • Failed to configure data port\n"; break;
            case -4: cerr << "  • Failed to send configuration file\n"; break;
            default: cerr << "  • Unknown error\n"; break;
        }
        return initResult;
    }

    // spawn all threads
    void* (*funcs[NUM_THREADS])(void*) = {
        sensor_thread,
        controller_thread,
        actuator_thread
    };
    for (int i = 0; i < NUM_THREADS; ++i) {
        if (pthread_create(&threads[i], nullptr, funcs[i], nullptr) != 0) {
            cerr << "[ERROR] pthread_create() for thread " << i << "\n";
            return -1;
        }
    }

    // join them (sensor_thread never returns, so program runs until killed)
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], nullptr);
    }

    return 0;
}
