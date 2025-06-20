#include "Radar.h"
#include <unistd.h>    // for sleep()
#include <algorithm>   // for std::min
#include <fstream>        // for file operations


IWR6843    sensor;
const int  NUM_THREADS = 3;
pthread_t  threads[NUM_THREADS];

ofstream csvFile("radar_output.csv");

void* sensor_thread(void* /*arg*/)
{
    if (!csvFile.is_open()) {
        cerr << "[ERROR] Could not open output file\n";
        pthread_exit(nullptr);
    }

    // Write header
    csvFile << "frame_id\tpoint_id\tx\ty\tz\tdoppler\tsnr\tnoise\n";
    

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

            uint32_t frame_id = hdr.getFrameNumber();
            size_t   pts      = pd.DetectedPoints_str.size();

            for (size_t i = 0; i < pts; ++i)
            {
                auto &pt   = pd.DetectedPoints_str[i];
                auto &side = pd.SideInfoPoint_str;
                auto &noise= pd.NoiseProfilePoint_str;

                // print tab‐separated columns:
                csvFile
                  << frame_id << '\t'
                  << (i+1)    << '\t'
                  << pt.x_f   << '\t'
                  << pt.y_f   << '\t'
                  << pt.z_f   << '\t'
                  << pt.doppler_f << '\t'
                  << side.snr    << '\t'
                  << noise.noisePoint
                  << '\n';
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

string formatTimestamp(const chrono::time_point<chrono::system_clock>& tp)
{
    auto dur = tp.time_since_epoch();
    auto s   = chrono::duration_cast<chrono::seconds>(dur);
    auto ns  = chrono::duration_cast<chrono::nanoseconds>(dur - s);
    time_t t = chrono::system_clock::to_time_t(tp);

    ostringstream oss;
    oss << put_time(localtime(&t), "%Y-%m-%d");
    oss << "." << setw(9) << setfill('0') << ns.count();
    return oss.str();
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
